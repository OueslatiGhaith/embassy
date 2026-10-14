//! Checks that QEMU's emulated DWC2 USB controller is visible to embassy.
//!
//! Reads the core ID and the root port status through the register definitions
//! in `embassy-usb-synopsys-otg`, and checks that the emulated USB stick shows up on the port.

#![no_std]
#![no_main]

use embassy_qemu_raspi2b_examples::exit;
use embassy_usb_synopsys_otg::otg_v1::Otg;

/// DWC2 OTG controller on the BCM2836 (peripheral base 0x3F00_0000).
const USB_BASE: usize = 0x3F98_0000;
/// Synopsys ID register; not part of the STM32-derived register block.
const GSNPSID: usize = 0x40;

#[embassy_executor::main(entry = "aarch32_rt::entry")]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let r = unsafe { Otg::from_ptr(USB_BASE as *mut ()) };

    let snpsid = unsafe { ((USB_BASE + GSNPSID) as *const u32).read_volatile() };
    defmt::info!("GSNPSID = {=u32:#010x}", snpsid);
    if snpsid & 0xFFFF_F000 != 0x4F54_2000 {
        defmt::error!("not a DWC2 core");
        exit(false);
    }

    defmt::info!("GINTSTS.CMOD (host mode) = {}", r.gintsts().read().cmod());

    let hprt = r.hprt().read();
    if hprt.pcsts() {
        defmt::info!("device connected, HPRT = {=u32:#010x}, speed = {}", hprt.0, hprt.pspd());
        exit(true);
    }
    defmt::error!("no device connected, HPRT = {=u32:#010x}", hprt.0);
    exit(false);
}
