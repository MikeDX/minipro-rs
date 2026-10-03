//! T48 protocol.
//!
//! The T48 shares the TL866II+ command set with minor differences.
//! This re-uses the TL866II+ implementation.

pub use super::tl866iiplus::Tl866iiPlusProtocol as T48Protocol;

/// Minimum firmware version for the T48.
///
/// The T48 runs its own firmware line, versioned independently of the
/// TL866II+ (e.g. `0x127` = 01.1.39) — it must not share
/// [`super::tl866iiplus::MIN_FIRMWARE`] (`0x255`), which rejects every
/// released T48 firmware at connect time.  Upstream C minipro expects
/// `0x127` and only warns on mismatch; it never refuses to connect, and
/// every released T48 firmware supports the shared command set.  So the
/// minimum is `0` (no hard floor), matching the TL866A/CS handling.
pub const MIN_FIRMWARE_T48: u32 = 0;
