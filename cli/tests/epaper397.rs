//! `--board waveshare-epaper397` with two real Arduino-ESP32 builds: this repository's GxEPD2
//! sketch (SPI2) and Waveshare's 02_E-Paper_Example (bit-banged GPIO). The images are not checked
//! in: `examples/waveshare-epaper397/build.sh DIR` builds them, `EPAPER397_FW_DIR=DIR` names them.
//! Pinned by the glass PNG (independent of build paths) and the controller report, not by
//! instruction counts.
#[path = "../../tests/common.rs"]
mod common;
use common::*;

const BIN: &str = env!("CARGO_BIN_EXE_esp32sim");

fn epaper(demo: &str, seconds: &str) -> (Run, Vec<u8>) {
    let dir = std::path::PathBuf::from(std::env::var("EPAPER397_FW_DIR")
        .expect("set EPAPER397_FW_DIR to the output of examples/waveshare-epaper397/build.sh")).join(demo);
    let (rom, png) = (rom("esp32s3_rev0"), tmp(&format!("{demo}.png")));
    let path = |f: &str| dir.join(f).to_str().unwrap().to_string();
    let r = run(BIN, &["--rom", rom.to_str().unwrap(), "--board", "waveshare-epaper397", "--boot", "rom", "--no-dump", "--console", "both",
        "--bootloader", &path("bootloader.bin"), "--ptable", &path("partitions.bin"), "--app", &path("app.bin"),
        "--tft-png", png.to_str().unwrap(), "--max-seconds", seconds]);
    let data = std::fs::read(&png).expect("png written");
    (r, data)
}

fn report(r: &Run) -> &str { r.stderr.lines().find(|l| l.starts_with("[emu] epaper:")).expect("epaper report line") }

/// GxEPD2 1.6.9: the initial clear and the frame are two standard full refreshes (4,278 ms BUSY
/// each), then three partial refreshes of the counter window (408 ms each), then hibernate.
#[test] #[ignore = "set EPAPER397_FW_DIR to the output of examples/waveshare-epaper397/build.sh"]
fn external_epaper397_gxepd2() {
    let (r, png) = epaper("gxepd2", "12");
    let ms = |prefix: &str| -> u64 {
        let line = r.stdout.lines().find(|l| l.starts_with(prefix)).unwrap_or_else(|| panic!("no {prefix:?} in\n{}", r.stdout));
        line.rsplit(' ').nth(1).unwrap().parse().unwrap()
    };
    let full = ms("epd: full refresh");
    assert!((2 * 4_278..2 * 4_278 + 400).contains(&full), "full refresh took {full} ms");
    for n in 1..=3 { let p = ms(&format!("epd: partial refresh {n}:")); assert!((408..460).contains(&p), "partial {n} took {p} ms"); }
    assert!(r.stdout.contains("epd: done"));
    let rep = report(&r);
    assert!(rep.contains("5 refreshes (full 2, fast 0, 4-grey 0, partial 3)") && rep.contains("asleep=true") && rep.contains("0 bit-banged, 0 dropped")
            && rep.contains("last window (280, 519, 299, 180) with driver output [df, 01, 02]"), "{rep}");
    expect_sha("epaper397-gxepd2.png.sha256", &png);
}

/// Waveshare's demo after its clear, the fast-init picture and the drawing (three full refreshes,
/// bit-banged): the drawing is on the glass.
#[test] #[ignore = "set EPAPER397_FW_DIR to the output of examples/waveshare-epaper397/build.sh"]
fn external_epaper397_waveshare_drawing() {
    let (r, png) = epaper("waveshare", "25");
    for line in ["EPD_3IN97_test Demo", "show image for array", "EPD_Display"] { assert!(r.stdout.contains(line), "no {line:?} in\n{}", r.stdout); }
    let rep = report(&r);
    assert!(rep.contains("3 refreshes (full 3, fast 0, 4-grey 0, partial 0)") && rep.contains("spi 0 bytes") && rep.contains("0 dropped"), "{rep}");
    expect_sha("epaper397-waveshare-drawing.png.sha256", &png);
}
