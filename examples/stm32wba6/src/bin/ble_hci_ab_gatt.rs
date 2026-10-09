#![no_std]
#![no_main]

use bt_hci::controller::ControllerCmdSync;
use cortex_m::peripheral::DWT;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{Config, bind_interrupts, rcc};
use embassy_stm32_wpan::bluetooth::gap::aci_gap;
use embassy_stm32_wpan::bluetooth::gatt::{
    CHAR_VALUE_HANDLE_OFFSET, CharProperties, CharacteristicHandle, GattEventMask, GattServer, SecurityPermissions,
    ServiceHandle, ServiceType, Uuid,
};
use embassy_stm32_wpan::bluetooth::{HCI, Normal};
use embassy_stm32_wpan::{HighInterruptHandler, LowInterruptHandler, Platform, new_platform};
use embassy_time::Timer;
use panic_probe as _;
use stm32wb_hci::aci::durations::{AdvInterval, PreferredConnInterval};
use stm32wb_hci::aci::gap::{GapSetDiscoverable, GapSetNonDiscoverable, GapUpdateAdvData};
use stm32wb_hci::aci::gatt::{GattAddChar, GattAddService, GattDelService, GattReadHandleValue, GattUpdateCharValue};
use stm32wb_hci::aci::ranges::EncKeySize;
use stm32wb_hci::aci::{flags as hci_flags, values as hci_values};
use stm32wb_hci::wire::Uuid as HciUuid;

/// Samples per command and path.
const N: usize = 256;

/// Characteristic value sizes for the update and read benchmarks. 240 keeps
/// the read response event within the 255 byte command buffer.
const SIZES: [usize; 4] = [1, 20, 100, 240];
const MAX_VALUE: usize = 240;

const SERVICE_UUID: [u8; 16] = [
    0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a, 0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0xab, 0xab,
];
const CHAR_UUID: [u8; 16] = [
    0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a, 0xe1, 0x11, 0x02, 0x00, 0x00, 0x00, 0xab, 0xab,
];
/// Attribute records per service: declaration and value of one characteristic, and its CCCD.
const MAX_ATTRIBUTE_RECORDS: u8 = 4;
const ENC_KEY_SIZE: u8 = 16;

/// `ADV_IND`, 100 ms interval, static random address (the `HCI::new` default).
const ADV_TYPE: u8 = 0x00;
const ADV_INTERVAL: u16 = 160;
const OWN_ADDR_TYPE: u8 = 0x01;
const LOCAL_NAME: &[u8] = b"\x09AB";
/// Manufacturer specific data, 10 bytes.
const MANUFACTURER_DATA: [u8; 10] = [0x09, 0xFF, 0x30, 0x00, 1, 2, 3, 4, 5, 6];

bind_interrupts!(struct Irqs {
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
});

#[embassy_executor::task]
async fn ble_runner_task(platform: &'static Platform) {
    platform.run_ble().await
}

#[inline(always)]
fn time<R>(f: impl FnOnce() -> R) -> (u32, R) {
    let start = DWT::cycle_count();
    let r = f();
    (DWT::cycle_count() - start, r)
}

#[inline(always)]
async fn time_async<R>(fut: impl Future<Output = R>) -> (u32, R) {
    let start = DWT::cycle_count();
    let r = fut.await;
    (DWT::cycle_count() - start, r)
}

/// Let the BLE runner process pending stack work before the next sample.
/// The sample loops never yield otherwise, so the runner would not run.
async fn settle() {
    Timer::after_millis(1).await;
}

struct Samples {
    ffi: [u32; N],
    hci: [u32; N],
}

impl Samples {
    const fn new() -> Self {
        Self {
            ffi: [0; N],
            hci: [0; N],
        }
    }

    fn report(&mut self, name: &str, sysclk_mhz: u32) {
        let a = stats(&mut self.ffi);
        let b = stats(&mut self.hci);
        info!(
            "{}: ffi min {} med {} max {} | hci min {} med {} max {} | med delta {} cyc ({} ns)",
            name,
            a.min,
            a.median,
            a.max,
            b.min,
            b.median,
            b.max,
            b.median as i32 - a.median as i32,
            (b.median as i32 - a.median as i32) * 1000 / sysclk_mhz as i32,
        );
    }
}

struct Stats {
    min: u32,
    median: u32,
    max: u32,
}

fn stats(samples: &mut [u32]) -> Stats {
    samples.sort_unstable();
    Stats {
        min: samples[0],
        median: samples[samples.len() / 2],
        max: samples[samples.len() - 1],
    }
}

/// Run `ffi` and `hci` `N` times each, alternating which goes first.
macro_rules! bench {
    ($samples:expr, $ffi:expr, $hci:expr) => {{
        for i in 0..N {
            settle().await;
            if i % 2 == 0 {
                $samples.ffi[i] = time(|| $ffi).0;
                $samples.hci[i] = time_async($hci).await.0;
            } else {
                $samples.hci[i] = time_async($hci).await.0;
                $samples.ffi[i] = time(|| $ffi).0;
            }
        }
    }};
}

/// Cycles to add a service, add a characteristic to it, and delete the service.
struct Lifecycle {
    add_service: u32,
    add_char: u32,
    del_service: u32,
}

fn ffi_lifecycle(gatt: &mut GattServer) -> Lifecycle {
    let (add_service, service) = time(|| {
        unwrap!(gatt.add_service(
            Uuid::from_u128_le(SERVICE_UUID),
            ServiceType::Primary,
            MAX_ATTRIBUTE_RECORDS
        ))
    });
    let (add_char, _) = time(|| {
        unwrap!(gatt.add_characteristic(
            service,
            Uuid::from_u128_le(CHAR_UUID),
            MAX_VALUE as u16,
            CharProperties::READ | CharProperties::NOTIFY,
            SecurityPermissions::NONE,
            GattEventMask::NONE,
            ENC_KEY_SIZE,
            true,
        ))
    });
    let (del_service, _) = time(|| unwrap!(gatt.delete_service(service)));
    Lifecycle {
        add_service,
        add_char,
        del_service,
    }
}

async fn hci_lifecycle(ble: &HCI<'_, Normal>) -> Lifecycle {
    let (add_service, service) = time_async(async {
        let cmd = GattAddService::new(
            HciUuid::Uuid128(SERVICE_UUID),
            hci_values::ServiceType::Primary,
            MAX_ATTRIBUTE_RECORDS,
        );
        unwrap!(ble.controller().exec(&cmd).await).service_handle
    })
    .await;
    let (add_char, _) = time_async(async {
        let cmd = GattAddChar::new(
            service,
            HciUuid::Uuid128(CHAR_UUID),
            MAX_VALUE as u16,
            hci_flags::CharProperties::READ | hci_flags::CharProperties::NOTIFY,
            hci_flags::SecurityPermissions::empty(),
            hci_flags::GattEventMask::empty(),
            unwrap!(EncKeySize::new(ENC_KEY_SIZE)),
            true,
        );
        unwrap!(ble.controller().exec(&cmd).await)
    })
    .await;
    let (del_service, _) =
        time_async(async { unwrap!(ble.controller().exec(&GattDelService::new(service)).await) }).await;
    Lifecycle {
        add_service,
        add_char,
        del_service,
    }
}

fn ffi_set_discoverable() {
    unwrap!(aci_gap::set_discoverable(
        ADV_TYPE,
        ADV_INTERVAL,
        ADV_INTERVAL,
        OWN_ADDR_TYPE,
        0,
        Some(LOCAL_NAME),
        None
    ));
}

fn hci_set_discoverable() -> GapSetDiscoverable<'static> {
    let interval = unwrap!(AdvInterval::from_units(ADV_INTERVAL));
    unwrap!(
        GapSetDiscoverable::try_new(
            hci_values::AdvertisingType::ConnectableUndirected,
            interval,
            interval,
            hci_values::OwnAddressType::StaticRandom,
            0,
            LOCAL_NAME,
            &[],
            PreferredConnInterval::OMITTED,
            PreferredConnInterval::OMITTED,
        )
        .ok()
    )
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = rcc::Config::new_wpan();
    let p = embassy_stm32::init(config);
    let sysclk_mhz = unwrap!(rcc::clocks(&p.RCC).sys.to_hertz()).0 / 1_000_000;

    let mut cp = unwrap!(cortex_m::Peripherals::take());
    cp.DCB.enable_trace();
    DWT::unlock();
    cp.DWT.enable_cycle_counter();

    info!("BLE GAP/GATT A/B benchmark: ffi = C command functions, hci = bt-hci via BleStack_Request");
    info!("sysclk {} MHz, {} samples per command and path", sysclk_mhz, N);

    let (platform, runtime) = new_platform!(8);
    spawner.spawn(unwrap!(ble_runner_task(platform)));

    let mut ble = unwrap!(HCI::new(platform, runtime, Irqs).await);
    let mut gatt = ble.gatt_server();

    // Service lifecycle: add a 128-bit service and characteristic, then delete
    // the service. Each sample is one call, alternating which path goes first.
    let mut add_service = Samples::new();
    let mut add_char = Samples::new();
    let mut del_service = Samples::new();
    for i in 0..N {
        settle().await;
        let (a, b) = if i % 2 == 0 {
            let a = ffi_lifecycle(&mut gatt);
            (a, hci_lifecycle(&ble).await)
        } else {
            let b = hci_lifecycle(&ble).await;
            (ffi_lifecycle(&mut gatt), b)
        };
        add_service.ffi[i] = a.add_service;
        add_char.ffi[i] = a.add_char;
        del_service.ffi[i] = a.del_service;
        add_service.hci[i] = b.add_service;
        add_char.hci[i] = b.add_char;
        del_service.hci[i] = b.del_service;
    }
    add_service.report("GattAddService (128-bit)", sysclk_mhz);
    add_char.report("GattAddChar (128-bit)", sysclk_mhz);
    del_service.report("GattDelService", sysclk_mhz);

    // A persistent characteristic for the value benchmarks.
    let service = unwrap!(gatt.add_service(
        Uuid::from_u128_le(SERVICE_UUID),
        ServiceType::Primary,
        MAX_ATTRIBUTE_RECORDS
    ));
    let char = unwrap!(gatt.add_characteristic(
        service,
        Uuid::from_u128_le(CHAR_UUID),
        MAX_VALUE as u16,
        CharProperties::READ | CharProperties::NOTIFY,
        SecurityPermissions::NONE,
        GattEventMask::NONE,
        ENC_KEY_SIZE,
        true,
    ));
    let value_handle = char.0 + CHAR_VALUE_HANDLE_OFFSET;

    let mut value = [0u8; MAX_VALUE];
    for (i, b) in value.iter_mut().enumerate() {
        *b = i as u8;
    }

    check_value_equivalence(&ble, &mut gatt, service, char, value_handle).await;

    let mut s = Samples::new();
    for len in SIZES {
        let v = &value[..len];
        info!("{} byte value:", len);

        bench!(
            s,
            unwrap!(gatt.update_characteristic_value(service, char, 0, v)),
            async {
                unwrap!(
                    ble.controller()
                        .exec(&unwrap!(GattUpdateCharValue::try_new(service.0, char.0, 0, v).ok()))
                        .await
                )
            }
        );
        s.report("GattUpdateCharValue", sysclk_mhz);

        let mut buf = [0u8; MAX_VALUE];
        bench!(s, unwrap!(gatt.read_value(value_handle, &mut buf[..len])), async {
            unwrap!(
                ble.controller()
                    .exec(&GattReadHandleValue::new(value_handle, 0, len as u16))
                    .await
            )
        });
        s.report("GattReadHandleValue", sysclk_mhz);
    }

    // Advertising start and stop. Each sample is one start or one stop,
    // alternating which path goes first.
    let mut start = Samples::new();
    let mut stop = Samples::new();
    for i in 0..N {
        settle().await;
        let ffi = |start: &mut Samples, stop: &mut Samples| {
            start.ffi[i] = time(ffi_set_discoverable).0;
            stop.ffi[i] = time(|| unwrap!(aci_gap::set_non_discoverable())).0;
        };
        if i % 2 == 0 {
            ffi(&mut start, &mut stop);
        }
        start.hci[i] = time_async(async { unwrap!(ble.controller().exec(&hci_set_discoverable()).await) })
            .await
            .0;
        stop.hci[i] = time_async(async { unwrap!(ble.controller().exec(&GapSetNonDiscoverable::new()).await) })
            .await
            .0;
        if i % 2 == 1 {
            ffi(&mut start, &mut stop);
        }
    }
    start.report("GapSetDiscoverable", sysclk_mhz);
    stop.report("GapSetNonDiscoverable", sysclk_mhz);

    // Advertising data update, while advertising.
    ffi_set_discoverable();
    bench!(s, unwrap!(aci_gap::update_adv_data(&MANUFACTURER_DATA)), async {
        unwrap!(
            ble.controller()
                .exec(&unwrap!(GapUpdateAdvData::try_new(&MANUFACTURER_DATA).ok()))
                .await
        )
    });
    unwrap!(aci_gap::set_non_discoverable());
    s.report("GapUpdateAdvData", sysclk_mhz);

    info!("Done.");

    cortex_m::asm::bkpt();
}

/// Check that a value written on one path reads back the same on the other.
async fn check_value_equivalence(
    ble: &HCI<'_, Normal>,
    gatt: &mut GattServer,
    service: ServiceHandle,
    char: CharacteristicHandle,
    value_handle: u16,
) {
    for len in SIZES {
        let mut written = [0u8; MAX_VALUE];
        for (i, b) in written[..len].iter_mut().enumerate() {
            *b = (i as u8) ^ 0x5A;
        }
        let written = &written[..len];

        // Write through hci, read through ffi.
        unwrap!(
            ble.controller()
                .exec(&unwrap!(
                    GattUpdateCharValue::try_new(service.0, char.0, 0, written).ok()
                ))
                .await
        );
        let mut buf = [0u8; MAX_VALUE];
        let n = unwrap!(gatt.read_value(value_handle, &mut buf));
        let hci_to_ffi = &buf[..n] == written;

        // Write through ffi, read through hci.
        let reversed = {
            let mut r = [0u8; MAX_VALUE];
            for (d, s) in r[..len].iter_mut().zip(written.iter().rev()) {
                *d = *s;
            }
            r
        };
        unwrap!(gatt.update_characteristic_value(service, char, 0, &reversed[..len]));
        let read = unwrap!(
            ble.controller()
                .exec(&GattReadHandleValue::new(value_handle, 0, MAX_VALUE as u16))
                .await
        );
        let ffi_to_hci = read.length as usize == len && &read.value[..] == &reversed[..len];

        if hci_to_ffi && ffi_to_hci {
            info!("equivalent: {} byte value", len);
        } else {
            warn!(
                "MISMATCH: {} byte value (hci -> ffi {}, ffi -> hci {})",
                len, hci_to_ffi, ffi_to_hci
            );
        }
    }
}
