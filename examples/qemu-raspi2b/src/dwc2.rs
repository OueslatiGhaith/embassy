use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_futures::yield_now;
use embassy_time::{Duration, Timer, with_timeout};
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator, UsbHostController, UsbPipe, pipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};
use embassy_usb_synopsys_otg::otg_v1::{Otg, regs, vals};

const USB_BASE: usize = 0x3F98_0000;
const CHANNELS: usize = 8;
const MAX_XFER: usize = 65536;
static CHANNELS_USED: AtomicU8 = AtomicU8::new(0);

fn otg() -> Otg {
    unsafe { Otg::from_ptr(USB_BASE as *mut ()) }
}

fn modify_hprt(f: impl FnOnce(&mut regs::Hprt)) {
    let mut v = otg().hprt().read();
    v.set_pena(false);
    v.set_pcdet(false);
    v.set_penchng(false);
    v.set_pocchng(false);
    f(&mut v);
    otg().hprt().write_value(v);
}

pub struct Dwc2Host {
    connected: bool,
}

impl Dwc2Host {
    pub fn new() -> Self {
        otg().gahbcfg().modify(|w| w.set_dmaen(true));
        Self { connected: false }
    }

    async fn reset_port(&mut self) -> Speed {
        modify_hprt(|w| w.set_prst(true));
        Timer::after_millis(50).await;
        modify_hprt(|w| w.set_prst(false));
        while !otg().hprt().read().pena() {
            yield_now().await;
        }
        // Reset recovery
        Timer::after_millis(10).await;

        match otg().hprt().read().pspd() {
            0 => Speed::High,
            1 => Speed::Full,
            _ => Speed::Low,
        }
    }
}

impl Default for Dwc2Host {
    fn default() -> Self {
        Self::new()
    }
}

impl<'d> UsbHostController<'d> for Dwc2Host {
    type Allocator = Dwc2Allocator;

    fn allocator(&self) -> Dwc2Allocator {
        Dwc2Allocator
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            let connected = otg().hprt().read().pcsts();
            if connected != self.connected {
                self.connected = connected;
                // Acknowledge the connect-detect change.
                modify_hprt(|w| w.set_pcdet(true));
                if !connected {
                    return DeviceEvent::Disconnected;
                }
                Timer::after_millis(100).await;
                return DeviceEvent::Connected(self.reset_port().await);
            }
            yield_now().await;
        }
    }

    async fn bus_reset(&mut self) {
        self.reset_port().await;
    }
}

#[derive(Clone, Copy)]
pub struct Dwc2Allocator;

impl<'d> UsbHostAllocator<'d> for Dwc2Allocator {
    type Pipe<T: pipe::Type, D: pipe::Direction> = Dwc2Pipe<T, D>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        if split.is_some() {
            return Err(HostError::Other("split transactions are not supported"));
        }
        let eptyp = match T::ep_type() {
            EndpointType::Control => vals::Eptyp::CONTROL,
            EndpointType::Bulk => vals::Eptyp::BULK,
            _ => return Err(HostError::Other("only control and bulk pipes are supported")),
        };

        let used = CHANNELS_USED
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |used| {
                (0..CHANNELS)
                    .find(|ch| used & (1 << ch) == 0)
                    .map(|ch| used | (1 << ch))
            })
            .map_err(|_| HostError::OutOfPipes)?;
        let ch = (0..CHANNELS).find(|ch| used & (1 << ch) == 0).unwrap();

        Ok(Dwc2Pipe {
            ch,
            addr,
            ep: endpoint.addr.index() as u8,
            mps: endpoint.max_packet_size,
            eptyp,
            timeout: TimeoutConfig::default(),
            _phantom: PhantomData,
        })
    }
}

pub struct Dwc2Pipe<T, D> {
    ch: usize,
    addr: u8,
    ep: u8,
    mps: u16,
    eptyp: vals::Eptyp,
    timeout: TimeoutConfig,
    _phantom: PhantomData<(T, D)>,
}

impl<T, D> Drop for Dwc2Pipe<T, D> {
    fn drop(&mut self) {
        halt_channel(self.ch);
        CHANNELS_USED.fetch_and(!(1 << self.ch), Ordering::AcqRel);
    }
}

fn halt_channel(ch: usize) {
    if otg().hcchar(ch).read().chena() {
        otg().hcchar(ch).modify(|w| w.set_chdis(true));
    }
}

/// Halts the channel if a transfer future is dropped mid-flight.
struct ChannelGuard(usize);

impl Drop for ChannelGuard {
    fn drop(&mut self) {
        halt_channel(self.0);
    }
}

impl<T, D> Dwc2Pipe<T, D> {
    async fn transaction(
        &mut self,
        dir_in: bool,
        pid: vals::Dpid,
        buf: *mut u8,
        len: usize,
    ) -> Result<usize, PipeError> {
        let r = otg();
        let ch = self.ch;
        let pktcnt = len.div_ceil(self.mps as usize).max(1);

        r.hcint(ch).write_value(regs::Hcint(0x7FF));
        r.hcdma(ch).write_value(regs::Hcdma(buf as u32));
        r.hctsiz(ch).write(|w| {
            w.set_xfrsiz(len as u32);
            w.set_pktcnt(pktcnt as u16);
            w.set_dpid(pid.to_bits());
        });
        let guard = ChannelGuard(ch);
        r.hcchar(ch).write(|w| {
            w.set_mpsiz(self.mps);
            w.set_epnum(self.ep);
            w.set_epdir(dir_in);
            w.set_eptyp(self.eptyp);
            w.set_dad(self.addr);
            w.set_chena(true);
        });

        let int = loop {
            let int = r.hcint(ch).read();
            if int.chh() {
                break int;
            }
            yield_now().await;
        };
        core::mem::forget(guard);

        if int.stall() {
            Err(PipeError::Stall)
        } else if int.bberr() {
            Err(PipeError::Babble)
        } else if int.txerr() {
            Err(PipeError::BadResponse)
        } else if !int.xfrc() {
            Err(PipeError::Canceled)
        } else {
            Ok(len - r.hctsiz(ch).read().xfrsiz() as usize)
        }
    }

    async fn transfer_in(&mut self, pid: vals::Dpid, buf: &mut [u8]) -> Result<usize, PipeError> {
        let mut done = 0;
        while done < buf.len() {
            let len = (buf.len() - done).min(MAX_XFER);
            let n = self.transaction(true, pid, buf[done..].as_mut_ptr(), len).await?;
            done += n;
            if n < len {
                break;
            }
        }
        Ok(done)
    }

    async fn transfer_out(&mut self, pid: vals::Dpid, buf: &[u8]) -> Result<(), PipeError> {
        for chunk in buf.chunks(MAX_XFER) {
            self.transaction(false, pid, chunk.as_ptr() as *mut u8, chunk.len())
                .await?;
        }
        Ok(())
    }

    async fn setup(&mut self, setup: &[u8; 8]) -> Result<(), PipeError> {
        self.transaction(false, vals::Dpid::MDATA, setup.as_ptr() as *mut u8, 8)
            .await
            .map(drop)
    }

    async fn status(&mut self, dir_in: bool) -> Result<(), PipeError> {
        self.transaction(dir_in, vals::Dpid::DATA1, core::ptr::null_mut(), 0)
            .await
            .map(drop)
    }
}

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for Dwc2Pipe<T, D> {
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        let timeout = if buf.is_empty() {
            self.timeout.no_data_timeout
        } else {
            self.timeout.data_timeout
        };
        with_timeout(Duration::try_from(timeout).unwrap(), async {
            self.setup(setup).await?;
            let n = if buf.is_empty() {
                0
            } else {
                self.transfer_in(vals::Dpid::DATA1, buf).await?
            };
            self.status(false).await?;
            Ok(n)
        })
        .await
        .unwrap_or(Err(PipeError::Timeout))
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        let timeout = if buf.is_empty() {
            self.timeout.no_data_timeout
        } else {
            self.timeout.data_timeout
        };
        with_timeout(Duration::try_from(timeout).unwrap(), async {
            self.setup(setup).await?;
            if !buf.is_empty() {
                self.transfer_out(vals::Dpid::DATA1, buf).await?;
            }
            self.status(true).await
        })
        .await
        .unwrap_or(Err(PipeError::Timeout))
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        self.transfer_in(vals::Dpid::DATA0, buf).await
    }

    async fn request_out(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        self.transfer_out(vals::Dpid::DATA0, buf).await?;
        if ensure_transaction_end && !buf.is_empty() && buf.len().is_multiple_of(self.mps as usize) {
            self.transaction(false, vals::Dpid::DATA0, core::ptr::null_mut(), 0)
                .await?;
        }
        Ok(())
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        // No toggle state: QEMU ignores DATA0/DATA1.
    }
}
