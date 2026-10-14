#![no_std]

use core::cell::RefCell;
use core::task::Waker;

use aarch32_cpu::register::{CntPct, Cntfrq};
use defmt_semihosting as _;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub mod dwc2;

/// Prints the panic and exits QEMU with a failure code.
#[panic_handler]
#[cfg(target_os = "none")]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("PANIC: {}", info);
    semihosting::process::exit(1);
}

/// Park every core except core 0; QEMU starts all four at the ELF entry point.
///
/// # Safety
///
/// Only called from the reset vector.
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.startup")]
pub unsafe extern "C" fn _start() {
    core::arch::naked_asm!(
        r#"
        mrc     p15, 0, r0, c0, c0, 5
        ands    r0, r0, 0xFF
        bne     1f
        ldr     pc, =_default_start
    1:
        wfe
        b       1b
    "#
    )
}

/// Time driver backed by the physical generic timer counter, without interrupts.
///
/// Expired timers are only woken from [`poll_timers`].
struct PolledTimeDriver {
    queue: Mutex<CriticalSectionRawMutex, RefCell<embassy_time_queue_utils::Queue>>,
}

impl embassy_time_driver::Driver for PolledTimeDriver {
    fn now(&self) -> u64 {
        // embassy-time runs at 1 MHz; CNTFRQ is whatever QEMU programmed.
        let ticks = CntPct::read().0 as u128;
        let freq = Cntfrq::read().0 as u128;
        (ticks * 1_000_000 / freq) as u64
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            self.queue.borrow(cs).borrow_mut().schedule_wake(at, waker);
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: PolledTimeDriver = PolledTimeDriver {
    queue: Mutex::new(RefCell::new(embassy_time_queue_utils::Queue::new())),
});

/// Wake expired timers forever. Run it next to the code under test.
pub async fn poll_timers() -> ! {
    use embassy_time_driver::Driver as _;

    loop {
        let now = DRIVER.now();
        critical_section::with(|cs| {
            DRIVER.queue.borrow(cs).borrow_mut().next_expiration(now);
        });
        embassy_futures::yield_now().await;
    }
}

/// Exit QEMU, reporting success or failure to the runner.
pub fn exit(ok: bool) -> ! {
    semihosting::process::exit(if ok { 0 } else { 1 });
}

defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());
