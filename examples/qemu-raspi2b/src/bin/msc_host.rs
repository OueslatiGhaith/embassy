//! MSC host test against QEMU's `usb-storage` device.
//!
//! Enumerates the stick on the root port, then runs the SCSI commands the MSC host driver
//! exposes and checks a write/read-back round trip. Exits QEMU with status 0 on success.

#![no_std]
#![no_main]

use embassy_futures::select::{Either, select};
use embassy_qemu_raspi2b_examples::dwc2::Dwc2Host;
use embassy_qemu_raspi2b_examples::{exit, poll_timers};
use embassy_time::{Duration, with_timeout};
use embassy_usb::class::msc::host::{MscDevice, MscError};
use embassy_usb::host::{BusRoute, BusState};

/// Size of the disk image the runner attaches (see `build.rs`).
const DISK_BYTES: u64 = 16 * 1024 * 1024;
const BLOCK: usize = 512;

#[embassy_executor::main(entry = "aarch32_rt::entry")]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let test = with_timeout(Duration::from_secs(30), run());
    match select(poll_timers(), test).await {
        Either::Second(Ok(Ok(()))) => {
            defmt::info!("MSC test passed");
            exit(true)
        }
        Either::Second(Ok(Err(e))) => {
            defmt::error!("MSC test failed: {}", e);
            exit(false)
        }
        Either::Second(Err(_)) => {
            defmt::error!("MSC test timed out");
            exit(false)
        }
    }
}

#[derive(defmt::Format)]
enum Error {
    Enumeration(embassy_usb::host::EnumerationError),
    Msc(MscError),
    Check(&'static str),
}

impl From<MscError> for Error {
    fn from(e: MscError) -> Self {
        Error::Msc(e)
    }
}

fn check(ok: bool, what: &'static str) -> Result<(), Error> {
    if ok { Ok(()) } else { Err(Error::Check(what)) }
}

async fn run() -> Result<(), Error> {
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb::host::bus(Dwc2Host::new(), &BUS_STATE);

    let speed = bus_ctrl.wait_for_connection().await;
    defmt::info!("device connected at {}", speed);

    let mut config = [0u8; 256];
    let (info, len) = bus
        .enumerate(BusRoute::Direct(speed), &mut config)
        .await
        .map_err(Error::Enumeration)?;
    defmt::info!(
        "enumerated: address {}, VID {=u16:04x} PID {=u16:04x}",
        info.device_address,
        info.device_desc.vendor_id,
        info.device_desc.product_id
    );

    let device = MscDevice::new(&bus, &info, &config[..len]).await?;
    defmt::info!("MSC interface {}, {} LUN(s)", device.interface(), device.num_luns());
    let mut lun = device.lun(0)?;

    let mut inquiry = [0u8; 36];
    let inq = lun.inquiry(&mut inquiry).await?;
    defmt::info!(
        "INQUIRY: vendor {=[u8]:a}, product {=[u8]:a}, removable {}",
        inq.vendor,
        inq.product,
        inq.removable
    );

    // The first TEST UNIT READY after reset reports UNIT ATTENTION.
    let mut ready = false;
    for _ in 0..3 {
        if lun.test_unit_ready().await? {
            ready = true;
            break;
        }
    }
    check(ready, "unit never became ready")?;

    let cap = lun.capacity().await?;
    defmt::info!("capacity: {} blocks of {} bytes", cap.block_count, cap.block_size);
    check(cap.block_size as usize == BLOCK, "unexpected block size")?;
    check(cap.block_count * BLOCK as u64 == DISK_BYTES, "unexpected block count")?;

    // Several blocks in one command, so the data phase spans many packets.
    const BLOCKS: usize = 8;
    let lba = 100;
    let mut written = [0u8; BLOCKS * BLOCK];
    for (i, b) in written.iter_mut().enumerate() {
        *b = (i * 7 + i / BLOCK) as u8;
    }
    lun.write_blocks(lba, &written).await?;

    let mut read = [0u8; BLOCKS * BLOCK];
    lun.read_blocks(lba, &mut read).await?;
    check(read == written, "read-back data differs from written data")?;
    defmt::info!("wrote and read back {} blocks at LBA {}", BLOCKS, lba);

    // A block nobody wrote must still be blank.
    let mut blank = [0xAAu8; BLOCK];
    lun.read_blocks(0, &mut blank).await?;
    check(blank.iter().all(|&b| b == 0), "LBA 0 is not blank")?;

    lun.flush().await?;
    Ok(())
}
