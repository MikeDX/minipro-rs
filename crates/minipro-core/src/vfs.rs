//! Optional in-memory files.
//!
//! Native reads and writes use the real filesystem until [`mount`] is called.
//! After that, mounted paths are served from memory. The WebAssembly build has
//! no filesystem, so every database and buffer file is mounted here first.

use std::collections::HashMap;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

static FILES: Mutex<Option<HashMap<PathBuf, Vec<u8>>>> = Mutex::new(None);

fn lock() -> std::sync::MutexGuard<'static, Option<HashMap<PathBuf, Vec<u8>>>> {
    FILES.lock().unwrap_or_else(|poison| poison.into_inner())
}

/// Remember `data` as the contents of `path`.
pub fn mount(path: &Path, data: impl Into<Vec<u8>>) {
    let mut files = lock();
    files
        .get_or_insert_with(HashMap::new)
        .insert(path.to_path_buf(), data.into());
}

/// Read `path` from a mount, or from the real filesystem on a native build.
pub fn read(path: &Path) -> io::Result<Vec<u8>> {
    let files = lock();
    if let Some(map) = files.as_ref() {
        if let Some(data) = map.get(path) {
            return Ok(data.clone());
        }
    }
    drop(files);
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::fs::read(path)
    }
    #[cfg(target_arch = "wasm32")]
    {
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("no mounted file {}", path.display()),
        ))
    }
}

/// Write `path`. Mounted sessions keep the bytes in memory.
pub fn write(path: &Path, data: &[u8]) -> io::Result<()> {
    let mut files = lock();
    if let Some(map) = files.as_mut() {
        map.insert(path.to_path_buf(), data.to_vec());
        return Ok(());
    }
    drop(files);
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::fs::write(path, data)
    }
    #[cfg(target_arch = "wasm32")]
    {
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            "filesystem is not mounted",
        ))
    }
}
