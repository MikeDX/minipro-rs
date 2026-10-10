//! USB transport layer.
//!
//! Abstracts over `nusb` bulk transfers so the rest of the crate never
//! touches raw USB primitives.  All transfers are made synchronous via
//! `pollster::block_on`.
//!
//! Endpoint map (from usb_nix.c):
//!
//! | Endpoint | Direction | Purpose                         |
//! |----------|-----------|---------------------------------|
//! | 0x01     | OUT       | Send command packet (64 B)      |
//! | 0x81     | IN        | Receive response packet (64 B)  |
//! | 0x02     | OUT       | Write payload, even 64-B blocks |
//! | 0x03     | OUT       | Write payload, odd 64-B blocks  |
//! | 0x82     | IN        | Read payload, even 64-B blocks  |
//! | 0x83     | IN        | Read payload, odd 64-B blocks   |
//! | 0x05     | OUT       | T76 write payload (single EP)   |
//! | 0x82     | IN        | T76 read payload                |

use std::sync::Arc;

use log::trace;
#[cfg(not(target_arch = "wasm32"))]
use nusb::transfer::RequestBuffer;
#[cfg(not(target_arch = "wasm32"))]
use nusb::{DeviceInfo, Interface};
#[cfg(not(target_arch = "wasm32"))]
use std::time::Duration;

use crate::{
    device::ProgrammerModel,
    error::{MiniproError, Result},
};

/// Timeout for outgoing USB transfers (sends, payload writes).  Matches
/// the C minipro's `MP_USBTIMEOUT` (5 seconds).  Sends are fast — the
/// firmware accepts data quickly even if processing it takes longer.
#[cfg(not(target_arch = "wasm32"))]
const USB_SEND_TIMEOUT: Duration = Duration::from_secs(5);

/// Timeout for incoming USB transfers (responses, payload reads).  Matches
/// the C minipro's `MP_USB_READ_TIMEOUT` (6 minutes).  The firmware may
/// take a long time to respond during slow operations like EPROM writes
/// (e.g. 27512 takes >5s per block to program before the status response
/// is ready).  Using the short send timeout here caused false timeouts on
/// slow EPROMs.
#[cfg(not(target_arch = "wasm32"))]
const USB_RECV_TIMEOUT: Duration = Duration::from_secs(360);

/// Race a future against a timer.  If the timer fires first, return a
/// timeout error.  Dropping the losing future cancels the pending USB
/// transfer (nusb cancels transfers when their futures are dropped).
#[cfg(not(target_arch = "wasm32"))]
fn block_on_with_timeout<F, T>(fut: F, timeout: Duration) -> Result<T>
where
    F: std::future::Future<Output = T> + std::marker::Unpin,
{
    let timer = timer_future(timeout);
    match pollster::block_on(futures::future::select(fut, timer)) {
        futures::future::Either::Left((result, _)) => Ok(result),
        futures::future::Either::Right(_) => Err(MiniproError::Protocol(
            "USB transfer timed out — the programmer may be in a bad state. \
             Unplug it, wait 20-30 seconds, plug it back in, and try again."
                .into(),
        )),
    }
}

/// Create a future that completes after `duration`.  Uses a background
/// thread + `futures::channel::oneshot` because pollster has no built-in
/// timer.
#[cfg(not(target_arch = "wasm32"))]
fn timer_future(duration: Duration) -> futures::channel::oneshot::Receiver<()> {
    let (tx, rx) = futures::channel::oneshot::channel();
    std::thread::spawn(move || {
        std::thread::sleep(duration);
        let _ = tx.send(());
    });
    rx
}

// ── USB VID/PID constants ────────────────────────────────────────────────────

const TL866_VID: u16 = 0x04d8;
const TL866_PID: u16 = 0xe11c;
const TL866II_VID: u16 = 0xa466;
const TL866II_PID: u16 = 0x0a53;
const T76_VID: u16 = 0xa466;
const T76_PID: u16 = 0x1a86;

#[allow(dead_code)]
const USB_TIMEOUT_MS: u32 = 5_000;

const CMD_EP_OUT: u8 = 0x01;
const CMD_EP_IN: u8 = 0x81;
const DATA_EP2_OUT: u8 = 0x02;
const DATA_EP3_OUT: u8 = 0x03;
const DATA_EP2_IN: u8 = 0x82;
const DATA_EP3_IN: u8 = 0x83;
const T76_DATA_EP_OUT: u8 = 0x05;

// ── Device open ──────────────────────────────────────────────────────────────

/// Identifies which USB VID/PID a `DeviceInfo` matches.
#[cfg(not(target_arch = "wasm32"))]
fn classify(info: &DeviceInfo) -> Option<ProgrammerModel> {
    let (vid, pid) = (info.vendor_id(), info.product_id());
    match (vid, pid) {
        (T76_VID, T76_PID) => Some(ProgrammerModel::T76),
        (TL866II_VID, TL866II_PID) => {
            // T56 and T48 share the same VID/PID as TL866II+; the model is
            // determined after opening by reading the system-info response.
            Some(ProgrammerModel::Tl866iiPlus)
        }
        (TL866_VID, TL866_PID) => Some(ProgrammerModel::Tl866a),
        _ => None,
    }
}

/// Synchronous bulk transport used by the WebUSB build.
///
/// Large TL866 payloads occupy endpoint 2 and endpoint 3 at the same time.
/// `transfer_out_pair` and `transfer_in_pair` must start both transfers before
/// waiting for either one.
pub trait HostBulk: Send + Sync {
    fn transfer_out(&self, endpoint: u8, data: &[u8]) -> Result<()>;
    fn transfer_in(&self, endpoint: u8, length: usize) -> Result<Vec<u8>>;
    fn transfer_out_pair(&self, ep_a: u8, a: &[u8], ep_b: u8, b: &[u8]) -> Result<()>;
    fn transfer_in_pair(
        &self,
        ep_a: u8,
        len_a: usize,
        ep_b: u8,
        len_b: usize,
    ) -> Result<(Vec<u8>, Vec<u8>)>;
}

/// A claimed USB interface ready for transfers.
pub struct UsbDevice {
    /// The USB PID, used to select T76-specific payload endpoints.
    pid: u16,
    host: Option<Arc<dyn HostBulk>>,
    #[cfg(not(target_arch = "wasm32"))]
    interface: Option<Interface>,
}

impl Default for UsbDevice {
    fn default() -> Self {
        Self {
            pid: 0,
            host: None,
            #[cfg(not(target_arch = "wasm32"))]
            interface: None,
        }
    }
}

impl UsbDevice {
    /// Return the USB product ID of this device.
    pub fn pid(&self) -> u16 {
        self.pid
    }

    /// Wrap a browser or test transport. `pid` selects the T76 payload endpoints.
    pub fn from_host(pid: u16, host: Arc<dyn HostBulk>) -> Self {
        Self {
            pid,
            host: Some(host),
            #[cfg(not(target_arch = "wasm32"))]
            interface: None,
        }
    }

    fn host(&self) -> Result<&dyn HostBulk> {
        self.host
            .as_deref()
            .ok_or_else(|| MiniproError::Protocol("USB device not open".into()))
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn iface(&self) -> Result<&Interface> {
        self.interface
            .as_ref()
            .ok_or_else(|| MiniproError::Protocol("USB device not open".into()))
    }
}

/// Map a programmer VID/PID to the model used before firmware identification.
///
/// T48 and T56 share the TL866II+ id. `get_system_info` replaces this guess.
pub fn model_from_ids(vid: u16, pid: u16) -> Option<ProgrammerModel> {
    match (vid, pid) {
        (T76_VID, T76_PID) => Some(ProgrammerModel::T76),
        (TL866II_VID, TL866II_PID) => Some(ProgrammerModel::Tl866iiPlus),
        (TL866_VID, TL866_PID) => Some(ProgrammerModel::Tl866a),
        _ => None,
    }
}

/// Poll USB until the programmer with the given PID disappears.
/// Returns `Ok(())` when disconnected or `Err` on timeout.
#[cfg(not(target_arch = "wasm32"))]
pub fn wait_for_disconnect(pid: u16, timeout_ms: u64) -> Result<()> {
    let start = std::time::Instant::now();
    while start.elapsed().as_millis() < timeout_ms as u128 {
        let devices = nusb::list_devices().map_err(MiniproError::Usb)?;
        let found = devices
            .filter(|d| classify(d).is_some() && d.product_id() == pid)
            .count();
        if found == 0 {
            return Ok(());
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    Err(MiniproError::Protocol(
        "Timed out waiting for programmer to disconnect".into(),
    ))
}

/// Poll USB until the programmer with the given PID reappears.
/// Returns `Ok(())` when reconnected or `Err` on timeout.
#[cfg(not(target_arch = "wasm32"))]
pub fn wait_for_reconnect(pid: u16, timeout_ms: u64) -> Result<()> {
    let start = std::time::Instant::now();
    while start.elapsed().as_millis() < timeout_ms as u128 {
        let devices = nusb::list_devices().map_err(MiniproError::Usb)?;
        let found = devices
            .filter(|d| classify(d).is_some() && d.product_id() == pid)
            .count();
        if found > 0 {
            return Ok(());
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    Err(MiniproError::Protocol(
        "Timed out waiting for programmer to reconnect".into(),
    ))
}

/// Open the first connected programmer.
///
/// Selects among TL866 (VID 0x04D8), TL866II+/T48/T56 (VID 0xA466 PID 0x0A53),
/// and T76 (VID 0xA466 PID 0x1A86).
#[cfg(not(target_arch = "wasm32"))]
pub fn open_programmer() -> Result<(UsbDevice, ProgrammerModel)> {
    let devices = nusb::list_devices().map_err(MiniproError::Usb)?;

    let mut found: Vec<(DeviceInfo, ProgrammerModel)> = devices
        .filter_map(|info| classify(&info).map(|model| (info, model)))
        .collect();

    match found.len() {
        0 => Err(MiniproError::NoProgrammerFound),
        1 => {
            let (info, model) = found.remove(0);
            let pid = info.product_id();
            let device = info.open().map_err(|e| {
                MiniproError::Protocol(format!(
                    "USB device found but cannot open it. \
                     This can happen after the computer wakes from sleep. \
                     Unplug the programmer, wait 20-30 seconds, plug it back in, and try again. \
                     If the problem persists, the WinUSB driver may need reinstallation \
                     with Zadig (https://zadig.akeo.ie/). (nusb error: {})",
                    e
                ))
            })?;

            // On macOS and Linux, the device may be left in USB configuration 0
            // (unconfigured) after a power-cycle. The C minipro (libusb)
            // detects this and calls SetConfiguration(1) before claiming the
            // interface. Without this, claim_interface fails with "interface
            // not found".
            //
            // This is not needed on Windows because WinUSB handles it, and
            // set_configuration is not supported per nusb docs.
            //
            // In nusb 0.1.14, set_configuration returns Result<(), Error>
            // directly (no MaybeFuture / .wait() — those were added in nusb
            // 0.2.x).
            #[cfg(not(target_os = "windows"))]
            {
                if device.active_configuration().is_err() {
                    device.set_configuration(1).map_err(|e| {
                        MiniproError::Protocol(format!(
                            "USB device is in configuration 0 (unconfigured) \
                             and cannot be set to configuration 1. \
                             This can happen after a power-cycle. \
                             Unplug the programmer, wait 20-30 seconds, \
                             plug it back in, and try again. (nusb error: {})",
                            e
                        ))
                    })?;
                }
            }

            let interface = device.claim_interface(0).map_err(|e| {
                MiniproError::Protocol(format!(
                    "USB device opened but cannot claim interface 0. \
                     This can happen after the computer wakes from sleep. \
                     Unplug the programmer, wait 20-30 seconds, plug it back in, and try again. \
                     If the problem persists, use Zadig to reinstall the WinUSB driver. (nusb error: {})",
                    e
                ))
            })?;
            Ok((
                UsbDevice {
                    interface: Some(interface),
                    host: None,
                    pid,
                },
                model,
            ))
        }
        _ => Err(MiniproError::MultipleProgrammersFound),
    }
}

#[cfg(target_arch = "wasm32")]
pub fn wait_for_disconnect(_pid: u16, _timeout_ms: u64) -> Result<()> {
    Err(MiniproError::Protocol(
        "WebUSB cannot see the programmer reconnect after it resets. Use the desktop app for firmware updates.".into(),
    ))
}

#[cfg(target_arch = "wasm32")]
pub fn wait_for_reconnect(_pid: u16, _timeout_ms: u64) -> Result<()> {
    Err(MiniproError::Protocol(
        "WebUSB cannot see the programmer reconnect after it resets. Use the desktop app for firmware updates.".into(),
    ))
}

#[cfg(target_arch = "wasm32")]
pub fn open_programmer() -> Result<(UsbDevice, ProgrammerModel)> {
    Err(MiniproError::Protocol(
        "Open the programmer with WebUSB before starting a session.".into(),
    ))
}

// ── Transfer primitives ──────────────────────────────────────────────────────

impl UsbDevice {
    /// Send a command packet on EP 0x01.
    ///
    /// Sends exactly `buf.len()` bytes — no padding. This matches the
    /// upstream C minipro behaviour, where different commands send
    /// different sizes (5, 8, 32, 48, 64 bytes). Padding short buffers
    /// to 64 bytes can cause the firmware to interpret the trailing
    /// zeros as a new command, desynchronising the protocol.
    pub fn msg_send(&self, buf: &[u8]) -> Result<()> {
        trace!(
            "msg_send: cmd=0x{:02x} len={} buf={:02x?}",
            buf.first().copied().unwrap_or(0),
            buf.len(),
            &buf[..buf.len().min(16)]
        );
        if self.host.is_some() {
            return self.host()?.transfer_out(CMD_EP_OUT, buf);
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let completion = block_on_with_timeout(
                self.iface()?.bulk_out(CMD_EP_OUT, buf.to_vec()),
                USB_SEND_TIMEOUT,
            )?;
            completion
                .status
                .map_err(|e| MiniproError::Protocol(e.to_string()))?;
            return Ok(());
        }
        #[cfg(target_arch = "wasm32")]
        {
            Err(MiniproError::Protocol("USB device not open".into()))
        }
    }

    /// Send an arbitrary-length packet to the command endpoint (EP 0x01).
    ///
    /// Unlike [`msg_send`], this does **not** pad or truncate the buffer.
    /// Used for T56/T76 FPGA bitstream upload where packets are larger than 64 B.
    pub fn msg_send_large(&self, buf: &[u8]) -> Result<()> {
        self.bulk_out_raw(CMD_EP_OUT, buf.to_vec())
    }

    /// Receive a response packet (64 bytes by default; pass a larger size for
    /// commands that return more).
    pub fn msg_recv(&self, size: usize) -> Result<Vec<u8>> {
        trace!("msg_recv: waiting for {size} bytes on EP 0x81");
        if self.host.is_some() {
            let data = self.host()?.transfer_in(CMD_EP_IN, size)?;
            trace!(
                "msg_recv: got {} bytes: {:02x?}",
                data.len(),
                &data[..data.len().min(16)]
            );
            return Ok(data);
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let completion = block_on_with_timeout(
                self.iface()?.bulk_in(CMD_EP_IN, RequestBuffer::new(size)),
                USB_RECV_TIMEOUT,
            )?;
            completion
                .status
                .map_err(|e| MiniproError::Protocol(e.to_string()))?;
            trace!(
                "msg_recv: got {} bytes: {:02x?}",
                completion.data.len(),
                &completion.data[..completion.data.len().min(16)]
            );
            return Ok(completion.data);
        }
        #[cfg(target_arch = "wasm32")]
        {
            Err(MiniproError::Protocol("USB device not open".into()))
        }
    }

    /// Write a payload to the chip.
    ///
    /// For payloads > 64 bytes the data is split across EP2/EP3 in the
    /// interleaved pattern expected by the TL866II+ firmware.
    pub fn write_payload(&self, data: &[u8]) -> Result<()> {
        self.write_payload_limit(data, 64)
    }

    pub fn write_payload_limit(&self, data: &[u8], limit: usize) -> Result<()> {
        // T76 uses a single endpoint for writes
        if self.pid == T76_PID {
            return self.bulk_out_raw(T76_DATA_EP_OUT, data.to_vec());
        }

        if data.len() <= limit {
            return self.bulk_out_raw(DATA_EP2_OUT, data.to_vec());
        }

        // Split into two halves for EP2 / EP3 (see usb_nix.c write_payload2).
        // Both transfers must be submitted concurrently — the firmware
        // expects data on both endpoints simultaneously.
        let (ep2_len, ep3_len) = split_lengths(data.len());
        let ep2_data = data[..ep2_len].to_vec();
        let ep3_data = data[ep2_len..ep2_len + ep3_len].to_vec();

        if self.host.is_some() {
            return self.host()?.transfer_out_pair(
                DATA_EP2_OUT,
                &ep2_data,
                DATA_EP3_OUT,
                &ep3_data,
            );
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            let iface = self.iface()?;
            let f2 = iface.bulk_out(DATA_EP2_OUT, ep2_data);
            let f3 = iface.bulk_out(DATA_EP3_OUT, ep3_data);
            let (c2, c3) = block_on_with_timeout(futures::future::join(f2, f3), USB_SEND_TIMEOUT)?;
            c2.status
                .map_err(|e| MiniproError::Protocol(e.to_string()))?;
            c3.status
                .map_err(|e| MiniproError::Protocol(e.to_string()))?;
            return Ok(());
        }
        #[cfg(target_arch = "wasm32")]
        {
            Err(MiniproError::Protocol("USB device not open".into()))
        }
    }

    /// Read a payload from the chip.
    pub fn read_payload(&self, length: usize) -> Result<Vec<u8>> {
        self.read_payload_limit(length, 64)
    }

    pub fn read_payload_limit(&self, length: usize, limit: usize) -> Result<Vec<u8>> {
        trace!("read_payload_limit: length={length} limit={limit}");
        // T76 uses EP 0x82 only for reads
        if self.pid == T76_PID {
            trace!("  -> T76 path: bulk_in(EP 0x82, {length})");
            return self.bulk_in_one(DATA_EP2_IN, length);
        }

        // Small reads: single EP2 transfer
        if length < 64 {
            trace!("  -> small path: bulk_in(EP 0x82, 64)");
            let data = self.bulk_in_one(DATA_EP2_IN, 64)?;
            if data.len() < length {
                return Err(MiniproError::Protocol(format!(
                    "USB error: short read on endpoint 0x82, got {} bytes, expected {length}",
                    data.len()
                )));
            }
            return Ok(data[..length].to_vec());
        }

        if length == 64 || length <= limit || limit == 0 {
            trace!("  -> single-EP2 path: bulk_in(EP 0x82, {length})");
            return self.bulk_in_one(DATA_EP2_IN, length);
        }

        // Large reads: interleaved EP2 + EP3, then de-interleave.
        // Both transfers must be submitted concurrently — the firmware
        // interleaves data across both endpoints and will block if only
        // one is being read.  Matches the C payload_transfer() which
        // submits both URBs then waits for both.
        let half = length / 2;
        trace!("  -> dual-EP path: concurrent bulk_in(EP 0x82, {half}) + bulk_in(EP 0x83, {half})");
        let (ep2, ep3) = if self.host.is_some() {
            self.host()?
                .transfer_in_pair(DATA_EP2_IN, half, DATA_EP3_IN, half)?
        } else {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let iface = self.iface()?;
                let f2 = iface.bulk_in(DATA_EP2_IN, RequestBuffer::new(half));
                let f3 = iface.bulk_in(DATA_EP3_IN, RequestBuffer::new(half));
                let (c2, c3) =
                    block_on_with_timeout(futures::future::join(f2, f3), USB_RECV_TIMEOUT)?;
                c2.status
                    .map_err(|e| MiniproError::Protocol(e.to_string()))?;
                c3.status
                    .map_err(|e| MiniproError::Protocol(e.to_string()))?;
                (c2.data, c3.data)
            }
            #[cfg(target_arch = "wasm32")]
            {
                return Err(MiniproError::Protocol("USB device not open".into()));
            }
        };
        if ep2.len() < half || ep3.len() < half {
            return Err(MiniproError::Protocol(format!(
                "USB error: short paired read, got {} and {} bytes, expected {half} each",
                ep2.len(),
                ep3.len()
            )));
        }
        Ok(deinterleave(&ep2[..half], &ep3[..half], length))
    }

    fn bulk_in_one(&self, endpoint: u8, length: usize) -> Result<Vec<u8>> {
        if self.host.is_some() {
            return self.host()?.transfer_in(endpoint, length);
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let c = block_on_with_timeout(
                self.iface()?.bulk_in(endpoint, RequestBuffer::new(length)),
                USB_RECV_TIMEOUT,
            )?;
            c.status
                .map_err(|e| MiniproError::Protocol(e.to_string()))?;
            return Ok(c.data);
        }
        #[cfg(target_arch = "wasm32")]
        {
            Err(MiniproError::Protocol("USB device not open".into()))
        }
    }

    fn bulk_out_raw(&self, ep: u8, data: Vec<u8>) -> Result<()> {
        trace!(
            "bulk_out_raw: ep=0x{:02x} len={} data={:02x?}",
            ep,
            data.len(),
            &data[..data.len().min(16)]
        );
        if self.host.is_some() {
            return self.host()?.transfer_out(ep, &data);
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let c = block_on_with_timeout(self.iface()?.bulk_out(ep, data), USB_SEND_TIMEOUT)?;
            c.status
                .map_err(|e| MiniproError::Protocol(e.to_string()))?;
            return Ok(());
        }
        #[cfg(target_arch = "wasm32")]
        {
            Err(MiniproError::Protocol("USB device not open".into()))
        }
    }
}

// ── Payload split/deinterleave helpers ───────────────────────────────────────

/// Compute (ep2_len, ep3_len) for a split payload write (mirrors usb_nix.c).
fn split_lengths(total: usize) -> (usize, usize) {
    let j = total % 128;
    if j != 0 {
        let k = (total - j) / 2;
        if j > 64 {
            (k + 64, j + k - 64)
        } else {
            (k, j + k)
        }
    } else {
        let half = total / 2;
        (half, half)
    }
}

/// Re-interleave data from EP2 and EP3 into contiguous buffer.
/// Even-numbered 64-byte blocks come from `ep2`, odd from `ep3`.
fn deinterleave(ep2: &[u8], ep3: &[u8], total: usize) -> Vec<u8> {
    let blocks = total / 64;
    let mut out = vec![0u8; total];
    let mut ep2_off = 0usize;
    let mut ep3_off = 0usize;

    for i in 0..blocks {
        let dst = i * 64;
        if i % 2 == 0 {
            out[dst..dst + 64].copy_from_slice(&ep2[ep2_off..ep2_off + 64]);
            ep2_off += 64;
        } else {
            out[dst..dst + 64].copy_from_slice(&ep3[ep3_off..ep3_off + 64]);
            ep3_off += 64;
        }
    }
    out
}

#[cfg(test)]
mod host_tests {
    use super::*;
    use std::sync::Mutex;

    struct Log(Mutex<Vec<&'static str>>);

    impl HostBulk for Log {
        fn transfer_out(&self, _endpoint: u8, _data: &[u8]) -> Result<()> {
            self.0.lock().unwrap().push("out");
            Ok(())
        }

        fn transfer_in(&self, _endpoint: u8, length: usize) -> Result<Vec<u8>> {
            self.0.lock().unwrap().push("in");
            Ok(vec![0xAB; length])
        }

        fn transfer_out_pair(&self, _ep_a: u8, a: &[u8], _ep_b: u8, b: &[u8]) -> Result<()> {
            assert_eq!(a.len(), 64);
            assert_eq!(b.len(), 64);
            self.0.lock().unwrap().push("outpair");
            Ok(())
        }

        fn transfer_in_pair(
            &self,
            _ep_a: u8,
            len_a: usize,
            _ep_b: u8,
            len_b: usize,
        ) -> Result<(Vec<u8>, Vec<u8>)> {
            self.0.lock().unwrap().push("inpair");
            Ok((vec![1; len_a], vec![2; len_b]))
        }
    }

    #[test]
    fn paired_payloads_use_one_dual_transfer() {
        let log = Arc::new(Log(Mutex::new(Vec::new())));
        let usb = UsbDevice::from_host(TL866II_PID, log.clone());
        usb.msg_send(&[0x01, 0x02]).unwrap();
        usb.write_payload(&[9, 8, 7]).unwrap();
        usb.write_payload(&vec![0u8; 128]).unwrap();
        let small = usb.read_payload(4).unwrap();
        assert_eq!(small, vec![0xAB; 4]);
        let wide = usb.read_payload(128).unwrap();
        assert_eq!(wide.len(), 128);
        assert_eq!(wide[0], 1);
        assert_eq!(wide[64], 2);
        assert_eq!(
            *log.0.lock().unwrap(),
            ["out", "out", "outpair", "in", "inpair"]
        );
    }
}
