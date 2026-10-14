use std::env;
use std::fs::{self, File};
use std::io::prelude::*;
use std::path::PathBuf;

/// Size of the disk image QEMU's usb-storage device is backed by.
const DISK_BYTES: u64 = 16 * 1024 * 1024;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    // Blank disk for the runner in `.cargo/config.toml`. QEMU opens it with
    // `snapshot=on`, so it stays blank.
    let target = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap()).join("target");
    fs::create_dir_all(&target).unwrap();
    File::create(target.join("stick.img"))
        .unwrap()
        .set_len(DISK_BYTES)
        .unwrap();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
