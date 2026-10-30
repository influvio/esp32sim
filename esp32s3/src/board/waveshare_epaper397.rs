//! Waveshare ESP32-S3-ePaper-3.97: an ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB PSRAM) with the
//! 3.97" 800×480 black/white GDEM0397T81 e-paper glass on an SSD1677 controller.
//!
//! Wiring (Waveshare's E-Paper tutorial, section 5.2, and `DEV_Config.h` of the board's
//! `02_E-Paper_Example`): BUSY GPIO 3 (high while busy), D/C 9, CS 10, SCLK 11, MOSI 12, RST 46;
//! no MISO, no display power switch. Waveshare's example bit-bangs SPI on these GPIOs; GxEPD2
//! (`GxEPD2_397_GDEM0397T81`) routes SPI2 to them through the GPIO matrix and drives CS itself.
//! Both reach the controller. Buttons, active low: BOOT GPIO 0, and the rotary key's three
//! directions GPIO 4 / 5 (press) / 6, as the CrossMux firmware's hardware notes for this board
//! (`docs/engineering/waveshare-epaper-397.md`) name them. The I2C bus (SDA 41, SCL 42) carries
//! register stubs for the AXP2101 PMIC (Waveshare calls it TG28) and the PCF85063 RTC; the
//! SHTC3, QMI8658, ES8311 codec and the TF card are not modelled.
use esp_soc::board::{BoardEdge, BoardModel, SpiPins, VirtualCycle};
use esp_soc::devices::{Ssd1677, SpiBitBang};
use esp_soc::devices::ssd1677::Refresh;

pub const PIN_EPD_BUSY: u8 = 3;
pub const PIN_EPD_DC: u8 = 9;
pub const PIN_EPD_CS: u8 = 10;
pub const PIN_EPD_SCLK: u8 = 11;
pub const PIN_EPD_MOSI: u8 = 12;
pub const PIN_EPD_RST: u8 = 46;
pub const PIN_BOOT: u8 = 0;
pub const PIN_KEY_UP: u8 = 4;
pub const PIN_KEY_PRESS: u8 = 5;
pub const PIN_KEY_DOWN: u8 = 6;
pub const EPD_WIDTH: usize = 800;
pub const EPD_HEIGHT: usize = 480;

pub struct WaveshareEpaper397 {
    /// the SSD1677; its `glass` is what the panel shows
    pub epd: Ssd1677,
    spi: SpiBitBang,
    /// CS (GPIO 10) and RST (GPIO 46) as the firmware drives them; both idle high
    pub cs_high: bool, pub rst_low: bool,
    pub gpio_events: u64,
    /// SPI bytes delivered to the controller / dropped (CS high, RST low, or not on GPIO 11/12)
    pub spi_bytes: u64, pub spi_dropped: u64,
    /// bytes assembled from bit-banged GPIO 11/12
    pub bitbang_bytes: u64,
}

impl Default for WaveshareEpaper397 { fn default() -> Self { Self::new() } }

impl WaveshareEpaper397 {
    pub fn new() -> Self {
        WaveshareEpaper397 { epd: Ssd1677::new(EPD_WIDTH, EPD_HEIGHT, PIN_EPD_BUSY, crate::periph::CPU_HZ / 1_000_000),
            spi: SpiBitBang::new(PIN_EPD_SCLK, PIN_EPD_MOSI, PIN_EPD_CS), cs_high: true, rst_low: false,
            gpio_events: 0, spi_bytes: 0, spi_dropped: 0, bitbang_bytes: 0 }
    }
    fn pins(&mut self, changes: &[(u8, bool)]) {
        for &(pin, level) in changes {
            self.gpio_events += 1;
            match pin {
                PIN_EPD_DC => self.epd.dc = level,
                PIN_EPD_CS => self.cs_high = level,
                PIN_EPD_RST => { if !level && !self.rst_low { self.epd.hw_reset(); } self.rst_low = !level; }
                _ => {}
            }
            if let Some(b) = self.spi.pin(pin, level) {
                if self.rst_low { self.spi_dropped += 1; } else { self.bitbang_bytes += 1; self.epd.byte(b); }
            }
        }
    }
    /// Whether a transaction with these routes is on the panel's wires. Routes the bus could not
    /// decode are taken as the panel's: it is the only SPI device on GPIO 11/12.
    fn on_panel_wires(pins: &SpiPins) -> bool {
        let routed = pins.sclk != 0 || pins.mosi != 0;
        !routed || (pins.sclk & (1 << PIN_EPD_SCLK) != 0 && pins.mosi & (1 << PIN_EPD_MOSI) != 0)
    }
}

impl BoardModel for WaveshareEpaper397 {
    fn name(&self) -> &'static str { "waveshare-epaper397" }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) { self.pins(changes); }
    fn gpio_output_at(&mut self, cycle: VirtualCycle, changes: &[(u8, bool)], _enabled: u64, _output: u64) {
        self.epd.advance_to(cycle);
        self.pins(changes);
    }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let selected = !self.cs_high || pins.cs & (1 << PIN_EPD_CS) != 0;
        if selected && !self.rst_low && Self::on_panel_wires(&pins) {
            self.spi_bytes += tx.len() as u64;
            for &b in tx { self.epd.byte(b); }
        } else {
            self.spi_dropped += tx.len() as u64;
        }
        vec![0xff; rx_len]   // SDO is not connected
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn crate::i2c::I2cDevice>)> {
        use crate::i2c::*;
        vec![
            (0, 0x34, Box::new(Reg8Device::new("axp2101-pmic-initialization-stub", &[(0x03, 0x4a)]))),
            (0, 0x51, Box::new(Reg8Device::new("pcf85063a-rtc-initialization-stub", &[]))),
        ]
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((EPD_WIDTH as u32, EPD_HEIGHT as u32, self.epd.rgb565(), self.epd.version())) }
    fn display_version(&self) -> u64 { self.epd.version() }
    fn display_frames(&self) -> u64 { self.epd.version() }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> { Some((self.epd.bw_ram_rgb565(), esp_soc::devices::ssd1677::RAM_COLS, esp_soc::devices::ssd1677::RAM_ROWS)) }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        vec![self.epd.busy_input(), (PIN_BOOT, true), (PIN_KEY_UP, true), (PIN_KEY_PRESS, true), (PIN_KEY_DOWN, true)]
    }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.epd.next_deadline() }
    fn advance_to(&mut self, cycle: VirtualCycle) { self.epd.advance_to(cycle); }
    fn take_edges(&mut self) -> Vec<BoardEdge> { self.epd.take_edges() }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name {
            "boot" | "back" | "btn1" => Some(PIN_BOOT),
            "up" | "left" => Some(PIN_KEY_UP),
            "press" | "function" | "sw" | "knob" => Some(PIN_KEY_PRESS),
            "down" | "right" => Some(PIN_KEY_DOWN),
            _ => None,
        }
    }
    fn report(&self) -> String {
        let e = &self.epd;
        let r = |k: Refresh| e.refreshes[k as usize];
        format!("[emu] epaper: {} refreshes (full {}, fast {}, 4-grey {}, partial {}), {} black pixels, last window {:?} with driver output {:02x?}, busy={} asleep={}; \
                 driver output now {:02x?} entry={:#04x} temp={:#04x} custom LUT={}; resets hw {} sw {}; {} RAM bytes; spi {} bytes, {} bit-banged, {} dropped; \
                 {} bytes while busy, {} in deep sleep, {} unknown commands; gpio events {}",
                e.version(), r(Refresh::Full), r(Refresh::Fast), r(Refresh::Gray4), r(Refresh::Partial), e.black_pixels(), e.last_window, e.last_driver, e.busy(), e.sleeping,
                e.driver, e.entry, e.temp, e.custom_lut, e.hw_resets, e.sw_resets, e.bytes_written, self.spi_bytes, self.bitbang_bytes, self.spi_dropped,
                e.bytes_while_busy, e.bytes_asleep, e.unknown_commands, self.gpio_events)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use emu_core::Bus;

    const US: u64 = crate::periph::CPU_HZ / 1_000_000;

    fn routes() -> SpiPins { SpiPins { sclk: 1 << PIN_EPD_SCLK, mosi: 1 << PIN_EPD_MOSI, miso: None, cs: 0 } }
    /// GxEPD2's `_writeCommand` / `_writeData` over SPI2 with CS on a GPIO.
    fn hw(b: &mut WaveshareEpaper397, c: u8, data: &[u8]) {
        b.gpio_changes(&[(PIN_EPD_DC, false), (PIN_EPD_CS, false)]); b.spi_transfer_pins(2, routes(), &[c], 0);
        b.gpio_changes(&[(PIN_EPD_CS, true)]);
        if !data.is_empty() {
            b.gpio_changes(&[(PIN_EPD_DC, true), (PIN_EPD_CS, false)]); b.spi_transfer_pins(2, routes(), data, 0);
            b.gpio_changes(&[(PIN_EPD_CS, true)]);
        }
    }
    /// Waveshare's `DEV_SPI_WriteByte`: CS low, eight MSB-first bits on MOSI with an SCLK pulse each, CS high.
    fn bitbang(b: &mut WaveshareEpaper397, dc: bool, byte: u8) {
        b.gpio_changes(&[(PIN_EPD_DC, dc), (PIN_EPD_CS, false)]);
        for i in 0..8 { b.gpio_changes(&[(PIN_EPD_MOSI, byte & (0x80 >> i) != 0), (PIN_EPD_SCLK, true), (PIN_EPD_SCLK, false)]); }
        b.gpio_changes(&[(PIN_EPD_CS, true)]);
    }
    fn finish(b: &mut WaveshareEpaper397) { let at = b.next_deadline().expect("a refresh is running"); b.advance_to(at); assert!(!b.epd.busy()); }

    #[test]
    fn hardware_spi_full_refresh_shows_after_busy() {
        let mut b = WaveshareEpaper397::new();
        hw(&mut b, 0x11, &[0x01]);
        hw(&mut b, 0x44, &[0, 0, 0x1f, 0x03]);
        hw(&mut b, 0x45, &[0xdf, 0x01, 0, 0]);
        hw(&mut b, 0x4e, &[0, 0]);
        hw(&mut b, 0x4f, &[0xdf, 0x01]);
        let mut img = vec![0xffu8; 100 * 480];
        img[0] = 0x7f;                                   // glass (0, 0) black
        hw(&mut b, 0x24, &img);
        hw(&mut b, 0x22, &[0xf7]);
        b.advance_to(10 * US);
        hw(&mut b, 0x20, &[]);
        assert_eq!(b.input_levels()[0], (PIN_EPD_BUSY, true));
        assert_eq!(b.display_version(), 0);
        finish(&mut b);
        let (w, h, px, version) = b.display().unwrap();
        assert_eq!((w, h, version, px[0], px[1]), (800, 480, 1, 0, 0xffff));
        assert_eq!(b.take_edges().iter().map(|e| (e.cycle, e.level)).collect::<Vec<_>>(), [(10 * US, true), ((10 + ssd_full()) * US, false)]);
        assert_eq!((b.spi_bytes, b.spi_dropped), (2 + 5 + 5 + 3 + 3 + 48_001 + 2 + 1, 0));
    }
    fn ssd_full() -> u64 { esp_soc::devices::ssd1677::FULL_US }

    #[test]
    fn bitbanged_bytes_reach_the_controller_and_the_reset_line_wakes_it() {
        let mut b = WaveshareEpaper397::new();
        bitbang(&mut b, false, 0x11); bitbang(&mut b, true, 0x05);
        assert_eq!((b.epd.entry, b.bitbang_bytes), (0x05, 2));
        bitbang(&mut b, false, 0x10); bitbang(&mut b, true, 0x01);
        assert!(b.epd.sleeping && b.input_levels()[0].1, "deep sleep holds BUSY high");
        b.gpio_changes(&[(PIN_EPD_RST, false)]);
        bitbang(&mut b, false, 0x11); bitbang(&mut b, true, 0x02);
        assert_eq!((b.epd.entry, b.spi_dropped), (0x03, 2), "held in reset: defaults, bytes dropped");
        b.gpio_changes(&[(PIN_EPD_RST, true)]);
        assert!(!b.epd.sleeping && !b.input_levels()[0].1);
        bitbang(&mut b, false, 0x11); bitbang(&mut b, true, 0x02);
        assert_eq!((b.epd.entry, b.epd.hw_resets), (0x02, 1));
    }

    #[test]
    fn deselected_or_foreign_routes_are_dropped() {
        let mut b = WaveshareEpaper397::new();
        b.gpio_changes(&[(PIN_EPD_DC, false)]);
        b.spi_transfer_pins(2, routes(), &[0x12], 0);
        assert_eq!((b.epd.sw_resets, b.spi_dropped), (0, 1), "CS high");
        b.spi_transfer_pins(2, SpiPins { cs: 1 << PIN_EPD_CS, ..routes() }, &[0x12], 0);
        assert_eq!(b.epd.sw_resets, 1, "a hardware CS on GPIO 10 selects the controller");
        b.gpio_changes(&[(PIN_EPD_CS, false)]);
        assert_eq!(b.spi_transfer_pins(2, SpiPins { sclk: 1 << 36, mosi: 1 << 35, miso: Some(37), cs: 0 }, &[0x12], 2), vec![0xff; 2]);
        assert_eq!((b.epd.sw_resets, b.spi_dropped), (1, 2), "the TF card's wires");
    }

    #[test]
    fn buttons_and_devices() {
        let mut b = WaveshareEpaper397::new();
        assert_eq!(b.input_levels(), vec![(3, false), (0, true), (4, true), (5, true), (6, true)]);
        assert_eq!((b.named_pin("boot"), b.named_pin("up"), b.named_pin("press"), b.named_pin("down"), b.named_pin("x")), (Some(0), Some(4), Some(5), Some(6), None));
        assert_eq!(b.i2c_devices().iter().map(|d| d.1).collect::<Vec<_>>(), [0x34, 0x51]);
        assert!(b.report().starts_with("[emu] epaper: 0 refreshes"));
        assert_eq!(crate::board::make_board("epaper397").unwrap().name(), "waveshare-epaper397");
    }

    /// The whole path through the S3 bus: Waveshare's bit-banged GPIO writes, BUSY read back from
    /// GPIO_IN while the refresh runs and after it. IDF v5.5.4 `soc/esp32s3/register/soc/gpio_reg.h`:
    /// OUT_W1TS 0x08, OUT_W1TC 0x0c, OUT1_W1TS 0x14, OUT1_W1TC 0x18, ENABLE_W1TS 0x24, ENABLE1_W1TS
    /// 0x30, IN 0x3c.
    #[test]
    fn bitbanged_refresh_through_the_bus_drives_busy() {
        const GPIO: u32 = 0x6000_4000;
        let mut bus = crate::bus::SocBus::new(1024, 1024, [0; 6]);
        bus.board = Box::new(WaveshareEpaper397::new());
        bus.attach_board_devices();
        assert!(bus.periph.i2c[0].has_device(0x34) && bus.periph.i2c[0].has_device(0x51));
        let low = (1 << PIN_EPD_DC) | (1 << PIN_EPD_CS) | (1 << PIN_EPD_SCLK) | (1 << PIN_EPD_MOSI);
        bus.write32(GPIO + 0x24, low).unwrap();
        bus.write32(GPIO + 0x30, 1 << (PIN_EPD_RST - 32)).unwrap();
        bus.write32(GPIO + 0x14, 1 << (PIN_EPD_RST - 32)).unwrap();
        bus.write32(GPIO + 0x08, 1 << PIN_EPD_CS).unwrap();
        let send = |bus: &mut crate::bus::SocBus, dc: bool, byte: u8| {
            bus.write32(GPIO + if dc { 0x08 } else { 0x0c }, 1 << PIN_EPD_DC).unwrap();
            bus.write32(GPIO + 0x0c, 1 << PIN_EPD_CS).unwrap();
            for i in 0..8 {
                bus.write32(GPIO + if byte & (0x80 >> i) != 0 { 0x08 } else { 0x0c }, 1 << PIN_EPD_MOSI).unwrap();
                bus.write32(GPIO + 0x08, 1 << PIN_EPD_SCLK).unwrap();
                bus.write32(GPIO + 0x0c, 1 << PIN_EPD_SCLK).unwrap();
            }
            bus.write32(GPIO + 0x08, 1 << PIN_EPD_CS).unwrap();
        };
        let busy = |bus: &mut crate::bus::SocBus| bus.read32(GPIO + 0x3c).unwrap() & (1 << PIN_EPD_BUSY) != 0;
        assert!(!busy(&mut bus));
        send(&mut bus, false, 0x24); send(&mut bus, true, 0x00);   // RAM row 0, glass row 479: 8 black pixels
        send(&mut bus, false, 0x22); send(&mut bus, true, 0xf7);
        send(&mut bus, false, 0x20);
        assert!(busy(&mut bus), "BUSY rises at Master Activation");
        Bus::tick(&mut bus, (esp_soc::devices::ssd1677::FULL_US * US / 2) as u32);
        assert!(busy(&mut bus));
        assert_eq!(bus.board.display_version(), 0);
        Bus::tick(&mut bus, (esp_soc::devices::ssd1677::FULL_US * US) as u32);
        assert!(!busy(&mut bus), "BUSY falls when the refresh ends");
        let (_, _, px, version) = bus.board.display().unwrap();
        assert_eq!(version, 1);
        assert_eq!(px.iter().filter(|&&p| p == 0).count(), 8);
        assert!(px[479 * 800..479 * 800 + 8].iter().all(|&p| p == 0));
    }
}
