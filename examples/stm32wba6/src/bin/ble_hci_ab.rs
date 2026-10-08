#![no_std]
#![no_main]

use bt_hci::cmd::controller_baseband::{Reset, SetEventMask};
use bt_hci::cmd::info::{ReadBdAddr, ReadLocalVersionInformation};
use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeSetEventMask, LeTestEnd, LeTransmitterTest};
use bt_hci::controller::ControllerCmdSync;
use bt_hci::param::{EventMask, LeEventMask};
use bt_hci::{FromHciBytes, WriteHci};
use cortex_m::peripheral::DWT;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{Config, bind_interrupts, rcc};
use embassy_stm32_wpan::bluetooth::HCI;
use embassy_stm32_wpan::bluetooth::hci::CommandSender;
use embassy_stm32_wpan::bluetooth::hci::types::DtmPacketPayload;
use embassy_stm32_wpan::{HighInterruptHandler, LowInterruptHandler, Platform, new_platform};
use embassy_time::Timer;
use panic_probe as _;
use stm32wb_hci::aci::hal::HalLeTxTestPacketNumber;

/// Samples per command and path.
const N: usize = 256;

const DTM_CHANNEL: u8 = 19;
const DTM_DATA_LENGTH: u8 = 37;

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

    info!("BLE HCI A/B benchmark: ffi = C command functions, hci = bt-hci via BleStack_Request");
    info!("sysclk {} MHz, {} samples per command and path", sysclk_mhz, N);

    let (platform, runtime) = new_platform!(8);
    spawner.spawn(unwrap!(ble_runner_task(platform)));

    let mut ble = unwrap!(HCI::new_dtm(platform, runtime, Irqs).await);
    let cmd = CommandSender::new();

    check_equivalence(&mut ble, &cmd).await;

    let all = EventMask::from_hci_bytes(&[0xFF; 8]).unwrap().0;
    let le_all = LeEventMask::from_hci_bytes(&[0xFF; 8]).unwrap().0;
    let mut s = Samples::new();

    bench!(s, unwrap!(cmd.read_local_version()), async {
        unwrap!(ble.controller().exec(&ReadLocalVersionInformation::new()).await)
    });
    s.report("ReadLocalVersionInformation", sysclk_mhz);

    bench!(s, unwrap!(cmd.read_bd_addr()), async {
        unwrap!(ble.controller().exec(&ReadBdAddr::new()).await)
    });
    s.report("ReadBdAddr", sysclk_mhz);

    bench!(s, unwrap!(cmd.le_read_local_supported_features()), async {
        unwrap!(ble.controller().exec(&LeReadLocalSupportedFeatures::new()).await)
    });
    s.report("LeReadLocalSupportedFeatures", sysclk_mhz);

    bench!(s, unwrap!(cmd.set_event_mask(u64::MAX)), async {
        unwrap!(ble.controller().exec(&SetEventMask::new(all)).await)
    });
    s.report("SetEventMask", sysclk_mhz);

    bench!(s, unwrap!(cmd.le_set_event_mask(u64::MAX)), async {
        unwrap!(ble.controller().exec(&LeSetEventMask::new(le_all)).await)
    });
    s.report("LeSetEventMask", sysclk_mhz);

    bench!(s, unwrap!(cmd.reset()), async {
        unwrap!(ble.controller().exec(&Reset::new()).await)
    });
    s.report("Reset", sysclk_mhz);

    // DTM start/stop, the commands timed by RF test sequencing. Each sample is
    // one start or one end, the other half of the pair is not timed.
    let mut start = Samples::new();
    let mut end = Samples::new();
    for i in 0..N {
        let tx = LeTransmitterTest::new(DTM_CHANNEL, DTM_DATA_LENGTH, DtmPacketPayload::Prbs9 as u8);

        start.ffi[i] = time(|| unwrap!(ble.dtm_transmit(DTM_CHANNEL, DTM_DATA_LENGTH, DtmPacketPayload::Prbs9))).0;
        end.ffi[i] = time(|| unwrap!(ble.dtm_end())).0;

        start.hci[i] = time_async(async { unwrap!(ble.controller().exec(&tx).await) }).await.0;
        end.hci[i] = time_async(async { unwrap!(ble.controller().exec(&LeTestEnd::new()).await) })
            .await
            .0;
    }
    start.report("LeTransmitterTest", sysclk_mhz);
    end.report("LeTestEnd", sysclk_mhz);

    // A vendor (ACI) command, measured while the radio is transmitting.
    unwrap!(ble.dtm_transmit(DTM_CHANNEL, DTM_DATA_LENGTH, DtmPacketPayload::Prbs9));
    bench!(s, unwrap!(ble.aci_hal_tx_test_packet_number()), async {
        unwrap!(ble.controller().exec(&HalLeTxTestPacketNumber::new()).await)
    });
    unwrap!(ble.dtm_end());
    s.report("AciHalLeTxTestPacketNumber", sysclk_mhz);

    info!("Done.");
    loop {
        Timer::after_secs(86400).await;
    }
}

/// Check that both paths return the same values.
async fn check_equivalence(ble: &mut HCI<'_, embassy_stm32_wpan::bluetooth::Test>, cmd: &CommandSender) {
    let a = unwrap!(cmd.read_local_version());
    let b = unwrap!(ble.controller().exec(&ReadLocalVersionInformation::new()).await);
    let mut hci_version = [0u8];
    let mut lmp_version = [0u8];
    unwrap!(b.hci_version.write_hci(&mut hci_version[..]));
    unwrap!(b.lmp_version.write_hci(&mut lmp_version[..]));
    check(
        "ReadLocalVersionInformation",
        a.hci_version == hci_version[0]
            && a.hci_revision == b.hci_subversion
            && a.lmp_version == lmp_version[0]
            && a.manufacturer_name == b.company_identifier
            && a.lmp_subversion == b.lmp_subversion,
    );

    let a = unwrap!(cmd.read_bd_addr());
    let b = unwrap!(ble.controller().exec(&ReadBdAddr::new()).await);
    check("ReadBdAddr", a == b.raw());

    let a = unwrap!(cmd.le_read_local_supported_features());
    let b = unwrap!(ble.controller().exec(&LeReadLocalSupportedFeatures::new()).await);
    let mut b_bytes = [0u8; 8];
    unwrap!(b.write_hci(&mut b_bytes[..]));
    check("LeReadLocalSupportedFeatures", a == b_bytes);

    // TX packet count, then LE Test End, while a transmitter test runs on each path.
    unwrap!(ble.dtm_transmit(DTM_CHANNEL, DTM_DATA_LENGTH, DtmPacketPayload::Prbs9));
    Timer::after_millis(100).await;
    let a_count = unwrap!(ble.aci_hal_tx_test_packet_number());
    let a_end = unwrap!(ble.dtm_end());

    let tx = LeTransmitterTest::new(DTM_CHANNEL, DTM_DATA_LENGTH, DtmPacketPayload::Prbs9 as u8);
    unwrap!(ble.controller().exec(&tx).await);
    Timer::after_millis(100).await;
    let b_count = unwrap!(ble.controller().exec(&HalLeTxTestPacketNumber::new()).await);
    let b_end = unwrap!(ble.controller().exec(&LeTestEnd::new()).await);

    info!(
        "100 ms TX test: ffi {} packets, hci {} packets",
        a_count, b_count.number_of_packets
    );
    // Same duration, so the counts should be within a few packets of each other.
    check(
        "AciHalLeTxTestPacketNumber",
        a_count.abs_diff(b_count.number_of_packets) < 16,
    );
    check("LeTestEnd", a_end == b_end);
}

fn check(name: &str, ok: bool) {
    if ok {
        info!("equivalent: {}", name);
    } else {
        warn!("MISMATCH: {}", name);
    }
}
