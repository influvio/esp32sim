//! The Solomon Systech SSD1677 e-paper controller over 4-wire SPI with a D/C line, as on the
//! 800×480 GDEM0397T81 glass. Commands, registers and power-on values follow the SSD1677
//! datasheet Rev 1.0 (Nov 2018), command table 7-1 and section 8.
//!
//! What is modelled: the black/white (0x24) and red (0x26) RAMs of 960×680 one-bit pixels, the
//! RAM window (0x44/0x45) and address counter (0x4E/0x4F) walked as the data entry mode (0x11)
//! says, Display Update Control 1/2 (0x21/0x22), Master Activation (0x20), the temperature
//! register (0x1A), software reset (0x12), the reset line and deep sleep (0x10), and BUSY.
//! The glass keeps what it shows until an activation with the DISPLAY stage (0x22 bit 2) has
//! finished: writing RAM alone changes nothing visible. Waveforms are not simulated; which RAM
//! bits a refresh shows is decided by the update sequence (see `activate`). Voltage, booster,
//! border, LUT (0x32) and OTP commands are accepted and their data ignored.
use crate::board::{BoardEdge, VirtualCycle};

/// RAM size: 960 source × 680 gate addresses (datasheet section 1; 0x44 XEnd and 0x45 YEnd POR).
pub const RAM_COLS: usize = 960;
pub const RAM_ROWS: usize = 680;
const STRIDE: usize = RAM_COLS / 8;

/// BUSY high times in µs. GxEPD2 1.6.9 `GxEPD2_397_GDEM0397T81.h` measured these on this glass:
/// power on 83,873, power off 138,810, full refresh 4,278,000 (standard waveform, 0x22 = 0xF7) and
/// 1,837,000 (fast, 0x1A = 0x6A then 0xD7), partial 408,000. The four-grey refresh has no
/// published time and takes the fast one. Software reset has none either: the datasheet's flow
/// (figure 9-1) waits 10 ms after it, GxEPD2 waits 10 ms without polling BUSY, so the model's BUSY
/// ends well before that.
pub const FULL_US: u64 = 4_278_000;
pub const FAST_US: u64 = 1_837_000;
pub const PARTIAL_US: u64 = 408_000;
pub const POWER_ON_US: u64 = 83_873;
pub const POWER_OFF_US: u64 = 138_810;
pub const SWRESET_US: u64 = 1_000;
/// An activation without the DISPLAY or analog stages (clock only, loading a LUT): not measured.
pub const OTHER_US: u64 = 100;

/// The temperature register value Waveshare's `EPD_3IN97_Init_4GRAY` writes (0x1A = 0x5A) to
/// select the four-grey waveform of the glass's OTP; `EPD_3IN97_Init_Fast` and GxEPD2 write 0x6A
/// for the fast one.
pub const TEMP_GRAY4: u8 = 0x5a;
/// What 0x22's "load temperature" stage reads from the internal sensor: 25 °C (0x1A's unit is
/// 1/16 °C for the 12-bit value, so the high byte is whole degrees).
pub const AMBIENT_TEMP: u8 = 25;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refresh { Full, Fast, Gray4, Partial }

pub struct Ssd1677 {
    width: usize, height: usize, busy_pin: u8, cycles_per_us: u64,
    /// the black/white and red RAMs, `row * 120 + column / 8`, bit 7 the leftmost pixel; 1 = white
    /// (BW) and 1 = red (RED, the previous image for this black/white glass)
    pub bw: Vec<u8>, pub red: Vec<u8>,
    /// what the glass shows, one grey level per pixel (0 black, 0xff white), row-major
    pub glass: Vec<u8>,
    /// the image a running refresh is driving the glass to, shown when BUSY falls
    pending: Option<(Vec<u8>, Refresh)>,
    /// the D/C line: low = command, high = data
    pub dc: bool,
    cmd: u8, args: [u8; 4], argn: usize,
    pub entry: u8, xs: u16, xe: u16, ys: u16, ye: u16, x: u16, y: u16,
    /// 0x01 Driver Output Control: MUX low, MUX high, GD/SM/TB
    pub driver: [u8; 3],
    pub update1: [u8; 2], pub update2: u8, pub temp: u8, pub border: u8, pub sensor: u8,
    /// 0x32 was written: the firmware brings its own waveform, which the model does not run
    pub custom_lut: bool,
    /// 0x37 F[6], RAM ping-pong for Display Mode 2: after such a refresh the BW RAM is copied to
    /// the RED RAM, so the next partial refresh compares against what is now shown
    pub ping_pong: bool,
    pub sleeping: bool,
    cycle: VirtualCycle, busy: bool, busy_until: Option<VirtualCycle>, edges: Vec<BoardEdge>,
    /// completed refreshes by kind (`Refresh as usize`); their sum is the display's change counter
    pub refreshes: [u64; 4],
    pub bytes_written: u64, pub hw_resets: u64, pub sw_resets: u64,
    /// bytes received while BUSY was high, which the datasheet says must not be sent
    pub bytes_while_busy: u64,
    /// bytes received in deep sleep (the interface is off until a hardware reset)
    pub bytes_asleep: u64,
    pub unknown_commands: u64,
    /// the RAM window and Driver Output Control of the last activation: (XStart, XEnd, YStart, YEnd)
    pub last_window: (u16, u16, u16, u16), pub last_driver: [u8; 3],
}

impl Ssd1677 {
    /// A controller behind a `width` × `height` glass (at most 960 × 680), BUSY on `busy_pin`,
    /// with board time counted at `cycles_per_us`.
    pub fn new(width: usize, height: usize, busy_pin: u8, cycles_per_us: u64) -> Self {
        assert!(width > 0 && width <= RAM_COLS && width.is_multiple_of(8) && height > 0 && height <= RAM_ROWS);
        let mut p = Ssd1677 { width, height, busy_pin, cycles_per_us, bw: vec![0xff; STRIDE * RAM_ROWS], red: vec![0; STRIDE * RAM_ROWS],
            glass: vec![0xff; width * height], pending: None, dc: false, cmd: 0, args: [0; 4], argn: 0,
            entry: 0, xs: 0, xe: 0, ys: 0, ye: 0, x: 0, y: 0, driver: [0; 3], update1: [0; 2], update2: 0, temp: 0, border: 0, sensor: 0,
            custom_lut: false, ping_pong: false, sleeping: false, cycle: 0, busy: false, busy_until: None, edges: Vec::new(),
            refreshes: [0; 4], bytes_written: 0, hw_resets: 0, sw_resets: 0, bytes_while_busy: 0, bytes_asleep: 0, unknown_commands: 0, last_window: (0, 0, 0, 0), last_driver: [0; 3] };
        p.registers_to_default();
        p
    }
    pub fn width(&self) -> usize { self.width }
    pub fn height(&self) -> usize { self.height }
    pub fn busy(&self) -> bool { self.busy }
    pub fn busy_input(&self) -> (u8, bool) { (self.busy_pin, self.busy) }
    /// Completed refreshes: the glass's change counter.
    pub fn version(&self) -> u64 { self.refreshes.iter().sum() }

    /// The SW-reset defaults (table 7-1 POR values; 0x12 keeps deep sleep, the RAM is unaffected).
    fn registers_to_default(&mut self) {
        self.entry = 0x03; self.xs = 0; self.xe = 0x3bf; self.ys = 0; self.ye = 0x2a7; self.x = 0; self.y = 0;
        self.driver = [0xa7, 0x02, 0x00]; self.update1 = [0, 0]; self.update2 = 0xff; self.temp = 0x7f; self.border = 0xc0; self.sensor = 0x48;
        self.custom_lut = false; self.ping_pong = false; self.cmd = 0; self.argn = 0;
    }

    /// RES# pulled low. The datasheet names a HW reset as the only way out of deep sleep (0x10) and
    /// lists it before the SW reset in the bring-up flow; it is taken to restore the same register
    /// defaults, with the RAM kept. A refresh still running is cut short: the glass keeps what it
    /// was being driven to.
    pub fn hw_reset(&mut self) {
        self.finish_refresh();
        self.registers_to_default();
        self.sleeping = false;
        self.busy_until = None;
        self.set_busy(false);
        self.hw_resets += 1;
    }

    fn set_busy(&mut self, level: bool) {
        if self.busy != level { self.busy = level; self.edges.push(BoardEdge { cycle: self.cycle, pin: self.busy_pin, level }); }
    }
    fn start_busy(&mut self, us: u64) {
        self.set_busy(true);
        self.busy_until = Some(self.cycle.saturating_add(us.saturating_mul(self.cycles_per_us)));
    }
    fn finish_refresh(&mut self) {
        if let Some((glass, kind)) = self.pending.take() { self.glass = glass; self.refreshes[kind as usize] += 1; }
    }

    pub fn next_deadline(&self) -> Option<VirtualCycle> { self.busy_until }
    /// Advance board time; BUSY falls (and a refresh lands on the glass) when its time is up.
    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        if let Some(at) = self.busy_until.filter(|&at| at <= cycle) {
            self.cycle = self.cycle.max(at);
            self.busy_until = None;
            self.finish_refresh();
            self.set_busy(false);
        }
        self.cycle = self.cycle.max(cycle);
    }
    pub fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }

    /// One byte clocked in while CS is low; `dc` says whether it is a command or data.
    pub fn byte(&mut self, b: u8) {
        if self.sleeping { self.bytes_asleep += 1; return; }
        if self.busy { self.bytes_while_busy += 1; }
        if !self.dc { self.command(b); return; }
        let n = self.argn;
        if n < 4 { self.args[n] = b; }
        self.argn = n.saturating_add(1);
        let a = self.args;
        let w = |lo: u8, hi: u8| u16::from(lo) | (u16::from(hi & 3) << 8);
        match (self.cmd, n) {
            (0x01, 0..=2) => self.driver[n] = b,
            (0x11, 0) => self.entry = b & 7,
            (0x44, 1) => self.xs = w(a[0], a[1]), (0x44, 3) => self.xe = w(a[2], a[3]),
            (0x45, 1) => self.ys = w(a[0], a[1]), (0x45, 3) => self.ye = w(a[2], a[3]),
            (0x4e, 0) => self.x = u16::from(b), (0x4e, 1) => self.x = w(a[0], a[1]),
            (0x4f, 0) => self.y = u16::from(b), (0x4f, 1) => self.y = w(a[0], a[1]),
            (0x21, 0..=1) => self.update1[n] = b,
            (0x22, 0) => self.update2 = b,
            (0x1a, 0) => self.temp = b,
            (0x18, 0) => self.sensor = b,
            (0x3c, 0) => self.border = b,
            (0x37, 5) => self.ping_pong = b & 0x40 != 0,
            (0x10, 0) => if b & 3 != 0 { self.enter_deep_sleep(); },
            (0x24, _) => self.write_ram(b, false),
            (0x26, _) => self.write_ram(b, true),
            _ => {}
        }
    }

    fn command(&mut self, b: u8) {
        self.cmd = b; self.argn = 0;
        match b {
            0x12 => { self.registers_to_default(); self.cmd = 0x12; self.sw_resets += 1; self.start_busy(SWRESET_US); }
            0x20 => self.activate(),
            0x32 => self.custom_lut = true,
            0x01 | 0x03 | 0x04 | 0x08 | 0x09 | 0x0a | 0x0c | 0x10 | 0x11 | 0x14 | 0x15 | 0x18 | 0x1a | 0x1b | 0x1c | 0x21 | 0x22 | 0x24 | 0x25 | 0x26 | 0x27
            | 0x28 | 0x29 | 0x2a | 0x2b | 0x2c | 0x2d | 0x2e | 0x2f | 0x31 | 0x34 | 0x35 | 0x36 | 0x37 | 0x38 | 0x39 | 0x3c | 0x41 | 0x44 | 0x45 | 0x46
            | 0x47 | 0x4d | 0x4e | 0x4f | 0x7f => {}
            _ => self.unknown_commands += 1,
        }
    }

    /// 0x10 with A[1:0] ≠ 0. The datasheet: "BUSY pad will keep output high" until a HW reset.
    fn enter_deep_sleep(&mut self) {
        self.finish_refresh();
        self.busy_until = None;
        self.set_busy(true);
        self.sleeping = true;
    }

    fn write_ram(&mut self, b: u8, red: bool) {
        let (col, row) = (usize::from(self.x >> 3), usize::from(self.y));
        if col < STRIDE && row < RAM_ROWS { if red { self.red[row * STRIDE + col] = b } else { self.bw[row * STRIDE + col] = b } }
        self.bytes_written += 1;
        let (x_up, y_up) = (self.entry & 1 != 0, self.entry & 2 != 0);
        if self.entry & 4 == 0 { if self.step_x(x_up) { self.step_y(y_up); } } else if self.step_y(y_up) { self.step_x(x_up); }
    }
    /// One byte (8 sources) along X; at the window's end the counter returns to XStart and the
    /// caller steps the other axis. The window compares bytes: X addresses are in sources.
    fn step_x(&mut self, up: bool) -> bool {
        let (c, end) = (usize::from(self.x >> 3), usize::from(self.xe >> 3));
        if c == end { self.x = self.xs; return true; }
        let next = if up { (c + 1) % STRIDE } else { (c + STRIDE - 1) % STRIDE };
        self.x = (next << 3) as u16;
        false
    }
    fn step_y(&mut self, up: bool) -> bool {
        let c = usize::from(self.y);
        if self.y == self.ye { self.y = self.ys; return true; }
        self.y = (if up { (c + 1) % RAM_ROWS } else { (c + RAM_ROWS - 1) % RAM_ROWS }) as u16;
        false
    }

    /// A RAM bit after 0x21's option for that RAM (A[3:0] BW, A[7:4] RED): 0100 bypass as 0, 1000 inverse.
    fn ram_bit(ram: &[u8], option: u8, col: usize, row: usize) -> bool {
        let bit = ram[row * STRIDE + col / 8] & (0x80 >> (col % 8)) != 0;
        match option & 0x0c { 0x04 => false, 0x08 => !bit, _ => bit }
    }

    /// 0x20, running the stages 0x22 enables (table 7-1): 0x80 clock on, 0x40 analog on, 0x20
    /// load the temperature, 0x10 load the LUT, 0x08 Display Mode 2, 0x04 DISPLAY, 0x02 analog
    /// off, 0x01 clock off. With DISPLAY the glass gets:
    /// - Display Mode 2 (0xFF Waveshare, 0xFC GxEPD2, partial): every pixel whose BW bit differs
    ///   from its RED bit goes to the BW colour; the rest is not driven and keeps what it shows.
    ///   The RED RAM stays as it was unless 0x37 enabled RAM ping-pong (Waveshare's driver for this
    ///   glass does not; GxEPD2 rewrites both RAMs after each partial refresh instead);
    /// - Display Mode 1 with the four-grey temperature value (0x1A = 0x5A, not reloaded): the BW
    ///   and RED bits as two planes, BW 1 / RED 1 white, BW 0 / RED 1 light grey, BW 1 / RED 0 dark
    ///   grey, 0 / 0 black (Waveshare's `EPD_3IN97_Display_4Gray`: GRAY1 blackest → 0/0, GRAY2 →
    ///   1/0, GRAY3 → 0/1, GRAY4 white → 1/1);
    /// - otherwise Display Mode 1, full (temperature loaded) or fast: the BW RAM.
    ///
    /// The glass maps RAM column x to glass column x and RAM row y to glass row `height - 1 - y`:
    /// "gates are reversed on this display" (GxEPD2, which therefore writes rows with Y
    /// decrementing from `height - 1`, as Waveshare's driver does).
    fn activate(&mut self) {
        let seq = self.update2;
        if seq & 0x20 != 0 { self.temp = AMBIENT_TEMP; }
        self.last_window = (self.xs, self.xe, self.ys, self.ye);
        self.last_driver = self.driver;
        if seq & 0x04 == 0 {
            self.start_busy(if seq & 0x02 != 0 { POWER_OFF_US } else if seq & 0x40 != 0 { POWER_ON_US } else { OTHER_US });
            return;
        }
        let kind = if seq & 0x08 != 0 { Refresh::Partial } else if seq & 0x20 != 0 { Refresh::Full } else if self.temp == TEMP_GRAY4 { Refresh::Gray4 } else { Refresh::Fast };
        self.finish_refresh();
        let (bw_opt, red_opt) = (self.update1[0] & 0x0f, self.update1[0] >> 4);
        let mut glass = self.glass.clone();
        for gy in 0..self.height {
            let row = self.height - 1 - gy;
            for gx in 0..self.width {
                let bw = Self::ram_bit(&self.bw, bw_opt, gx, row);
                let red = Self::ram_bit(&self.red, red_opt, gx, row);
                let px = &mut glass[gy * self.width + gx];
                *px = match kind {
                    Refresh::Partial => if bw != red { if bw { 0xff } else { 0 } } else { *px },
                    Refresh::Gray4 => match (bw, red) { (true, true) => 0xff, (false, true) => 0xaa, (true, false) => 0x55, (false, false) => 0 },
                    Refresh::Full | Refresh::Fast => if bw { 0xff } else { 0 },
                };
            }
        }
        self.pending = Some((glass, kind));
        if kind == Refresh::Partial && self.ping_pong { self.red.copy_from_slice(&self.bw); }
        self.start_busy(match kind { Refresh::Full => FULL_US, Refresh::Fast | Refresh::Gray4 => FAST_US, Refresh::Partial => PARTIAL_US });
    }

    /// The glass as RGB565 greys.
    pub fn rgb565(&self) -> Vec<u16> {
        self.glass.iter().map(|&g| { let g = u16::from(g); ((g >> 3) << 11) | ((g >> 2) << 5) | (g >> 3) }).collect()
    }
    /// The BW RAM as RGB565 (white 0xffff, black 0), all 960 × 680 addresses unflipped.
    pub fn bw_ram_rgb565(&self) -> Vec<u16> {
        (0..RAM_ROWS * RAM_COLS).map(|i| if self.bw[i / 8] & (0x80 >> (i % 8)) != 0 { 0xffff } else { 0 }).collect()
    }
    /// Black pixels on the glass, for reports and tests.
    pub fn black_pixels(&self) -> usize { self.glass.iter().filter(|&&g| g < 0x80).count() }
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 800;
    const H: usize = 480;

    fn panel() -> Ssd1677 { Ssd1677::new(W, H, 3, 1) }
    fn cmd(p: &mut Ssd1677, c: u8, data: &[u8]) {
        p.dc = false; p.byte(c);
        p.dc = true; for &b in data { p.byte(b); }
    }
    fn wait(p: &mut Ssd1677) { if let Some(at) = p.next_deadline() { p.advance_to(at); } assert!(!p.busy()); }
    fn lo_hi(v: u16) -> [u8; 2] { [v as u8, (v >> 8) as u8] }
    /// GxEPD2's `_setPartialRamArea`: Y reversed, entry mode X+ Y-.
    fn gx_window(p: &mut Ssd1677, x: u16, y: u16, w: u16, h: u16) {
        let y = H as u16 - y - h;
        cmd(p, 0x11, &[0x01]);
        let (xs, xe, ys, ye) = (lo_hi(x), lo_hi(x + w - 1), lo_hi(y + h - 1), lo_hi(y));
        cmd(p, 0x44, &[xs[0], xs[1], xe[0], xe[1]]);
        cmd(p, 0x45, &[ys[0], ys[1], ye[0], ye[1]]);
        cmd(p, 0x4e, &xs);
        cmd(p, 0x4f, &ys);
    }
    /// A white `w`×`h` image with a black pixel at each of `dots`, row-major 1 bpp.
    fn image(w: usize, h: usize, dots: &[(usize, usize)]) -> Vec<u8> {
        let mut img = vec![0xff; w / 8 * h];
        for &(x, y) in dots { img[y * (w / 8) + x / 8] &= !(0x80 >> (x % 8)); }
        img
    }
    fn glass(p: &Ssd1677, x: usize, y: usize) -> u8 { p.glass[y * W + x] }

    #[test]
    fn gxepd2_full_refresh_lands_upright_after_busy() {
        let mut p = panel();
        cmd(&mut p, 0x12, &[]);
        assert!(p.busy());
        wait(&mut p);
        gx_window(&mut p, 0, 0, W as u16, H as u16);
        let img = image(W, H, &[(0, 0), (799, 0), (0, 479), (799, 479), (123, 45)]);
        cmd(&mut p, 0x24, &img);
        assert_eq!(p.black_pixels(), 0, "RAM writes alone do not change the glass");
        cmd(&mut p, 0x21, &[0x40, 0x00]);
        cmd(&mut p, 0x22, &[0xf7]);
        p.advance_to(1000);
        cmd(&mut p, 0x20, &[]);
        assert_eq!(p.take_edges(), [BoardEdge { cycle: 0, pin: 3, level: true }, BoardEdge { cycle: SWRESET_US, pin: 3, level: false }, BoardEdge { cycle: 1000, pin: 3, level: true }]);
        assert_eq!(p.next_deadline(), Some(1000 + FULL_US));
        p.advance_to(1000 + FULL_US - 1);
        assert_eq!((p.busy(), p.black_pixels()), (true, 0), "the glass changes when the refresh ends");
        p.advance_to(1000 + FULL_US);
        assert_eq!(p.take_edges(), [BoardEdge { cycle: 1000 + FULL_US, pin: 3, level: false }]);
        assert_eq!(p.black_pixels(), 5);
        for (x, y) in [(0, 0), (799, 0), (0, 479), (799, 479), (123, 45)] { assert_eq!(glass(&p, x, y), 0, "({x}, {y})"); }
        assert_eq!((p.refreshes, p.version()), ([1, 0, 0, 0], 1));
    }

    #[test]
    fn gxepd2_partial_window_drives_only_changed_pixels() {
        let mut p = panel();
        gx_window(&mut p, 0, 0, W as u16, H as u16);
        cmd(&mut p, 0x26, &vec![0xff; W / 8 * H]);
        cmd(&mut p, 0x24, &vec![0xff; W / 8 * H]);
        // the counter digits: a 16×2 window at (280, 180) with two black pixels
        gx_window(&mut p, 280, 180, 16, 2);
        cmd(&mut p, 0x24, &image(16, 2, &[(0, 0), (15, 1)]));
        cmd(&mut p, 0x21, &[0x00, 0x00]);
        cmd(&mut p, 0x22, &[0xfc]);
        cmd(&mut p, 0x20, &[]);
        assert_eq!(p.next_deadline(), Some(PARTIAL_US));
        wait(&mut p);
        assert_eq!((glass(&p, 280, 180), glass(&p, 295, 181), p.black_pixels()), (0, 0, 2));
        assert_eq!(p.refreshes[Refresh::Partial as usize], 1);
        assert_eq!(p.last_window, (280, 295, 299, 298));
        // a partial refresh with RED equal to BW leaves the glass alone, even where it differs
        p.glass[0] = 0;
        cmd(&mut p, 0x20, &[]);
        wait(&mut p);
        assert_eq!((glass(&p, 0, 0), glass(&p, 280, 180)), (0, 0));
    }

    #[test]
    fn waveshare_full_image_wraps_the_first_row_to_the_bottom() {
        // EPD_3IN97_Init: entry X+ Y-, window Y 479..0, counter (0, 0): row 0 goes to RAM row 0, the
        // window's end, so the counter wraps to 479 and the remaining rows follow downwards.
        let mut p = panel();
        cmd(&mut p, 0x01, &[0xdf, 0x01, 0x02]);
        cmd(&mut p, 0x11, &[0x01]);
        cmd(&mut p, 0x44, &[0, 0, 0x1f, 0x03]);
        cmd(&mut p, 0x45, &[0xdf, 0x01, 0, 0]);
        cmd(&mut p, 0x4e, &[0, 0]);
        cmd(&mut p, 0x4f, &[0, 0]);
        cmd(&mut p, 0x24, &image(W, H, &[(5, 0), (6, 1), (7, 479)]));
        cmd(&mut p, 0x22, &[0xf7]);
        cmd(&mut p, 0x20, &[]);
        wait(&mut p);
        assert_eq!((glass(&p, 5, 479), glass(&p, 6, 0), glass(&p, 7, 478), p.black_pixels()), (0, 0, 0, 3));
        assert_eq!((p.x, p.y), (0, 0), "48000 bytes walk the whole window back to the start");
        assert_eq!(p.driver, [0xdf, 0x01, 0x02]);
    }

    #[test]
    fn entry_modes_walk_the_window_in_every_direction() {
        for entry in 0..8u8 {
            let mut p = panel();
            cmd(&mut p, 0x11, &[entry]);
            let (x_up, y_up) = (entry & 1 != 0, entry & 2 != 0);
            let (xs, xe) = if x_up { (16u16, 31u16) } else { (31, 16) };
            let (ys, ye) = if y_up { (10u16, 12u16) } else { (12, 10) };
            cmd(&mut p, 0x44, &[xs as u8, 0, xe as u8, 0]);
            cmd(&mut p, 0x45, &[ys as u8, 0, ye as u8, 0]);
            cmd(&mut p, 0x4e, &[xs as u8, 0]);
            cmd(&mut p, 0x4f, &[ys as u8, 0]);
            cmd(&mut p, 0x24, &[1, 2, 3, 4, 5, 6, 7]);
            let at = |col: usize, row: usize| p.bw[row * STRIDE + col];
            let cols = if x_up { [2, 3] } else { [3, 2] };
            let rows: [usize; 3] = if y_up { [10, 11, 12] } else { [12, 11, 10] };
            let expect: Vec<(usize, usize)> = if entry & 4 == 0 { rows.iter().flat_map(|&r| cols.iter().map(move |&c| (c, r))).collect() }
                                              else { cols.iter().flat_map(|&c| rows.iter().map(move |&r| (c, r))).collect() };
            for (i, &(c, r)) in expect.iter().enumerate().skip(1) { assert_eq!(at(c, r), i as u8 + 1, "entry {entry:#x} byte {i}"); }
            assert_eq!(at(cols[0], rows[0]), 7, "entry {entry:#x}: the seventh byte wraps to the window start");
        }
    }

    #[test]
    fn out_of_range_addresses_do_not_panic() {
        let mut p = panel();
        cmd(&mut p, 0x44, &[0xff, 0xff, 0xff, 0xff]);
        cmd(&mut p, 0x45, &[0xff, 0xff, 0xff, 0xff]);
        cmd(&mut p, 0x4e, &[0xff, 0xff]);
        cmd(&mut p, 0x4f, &[0xff, 0xff]);
        for entry in 0..8 { cmd(&mut p, 0x11, &[entry]); cmd(&mut p, 0x24, &[0; 300]); cmd(&mut p, 0x26, &[0; 300]); }
        cmd(&mut p, 0x01, &[1, 2, 3, 4, 5, 6]);
        cmd(&mut p, 0x99, &[1, 2]);
        assert_eq!(p.unknown_commands, 1);
        cmd(&mut p, 0x22, &[0xff]);
        cmd(&mut p, 0x20, &[]);
        wait(&mut p);
    }

    #[test]
    fn four_grey_combines_both_rams() {
        let mut p = panel();
        gx_window(&mut p, 0, 0, W as u16, H as u16);
        // pixels 0..3 of row 0: BW 1 0 1 0, RED 1 1 0 0 → white, light, dark, black
        let mut bw = vec![0xff; W / 8 * H]; let mut red = vec![0xff; W / 8 * H];
        bw[0] = 0b1010_1111; red[0] = 0b1100_1111;
        cmd(&mut p, 0x24, &bw);
        cmd(&mut p, 0x26, &red);
        cmd(&mut p, 0x1a, &[TEMP_GRAY4]);
        cmd(&mut p, 0x22, &[0xd7]);
        cmd(&mut p, 0x20, &[]);
        assert_eq!(p.next_deadline(), Some(FAST_US));
        wait(&mut p);
        assert_eq!([glass(&p, 0, 0), glass(&p, 1, 0), glass(&p, 2, 0), glass(&p, 3, 0), glass(&p, 4, 0)], [0xff, 0xaa, 0x55, 0, 0xff]);
        assert_eq!(p.refreshes[Refresh::Gray4 as usize], 1);
        // the same with the fast temperature is black and white from the BW RAM
        cmd(&mut p, 0x1a, &[0x6a]);
        cmd(&mut p, 0x20, &[]);
        wait(&mut p);
        assert_eq!([glass(&p, 0, 0), glass(&p, 1, 0), glass(&p, 2, 0), glass(&p, 3, 0)], [0xff, 0, 0xff, 0]);
        assert_eq!(p.refreshes, [0, 1, 1, 0]);
        // a full refresh loads the sensor's temperature, so a later 0xD7 is fast, not grey
        cmd(&mut p, 0x1a, &[TEMP_GRAY4]);
        cmd(&mut p, 0x22, &[0xf7]); cmd(&mut p, 0x20, &[]); wait(&mut p);
        cmd(&mut p, 0x22, &[0xd7]); cmd(&mut p, 0x20, &[]); wait(&mut p);
        assert_eq!((p.temp, p.refreshes), (AMBIENT_TEMP, [1, 2, 1, 0]));
    }

    #[test]
    fn partial_refreshes_compare_against_red_unless_ping_pong() {
        for ping_pong in [false, true] {
            let mut p = panel();
            if ping_pong { cmd(&mut p, 0x37, &[0, 0, 0, 0, 0, 0x40, 0, 0, 0, 0]); }
            gx_window(&mut p, 0, 0, W as u16, H as u16);
            cmd(&mut p, 0x26, &vec![0xff; W / 8 * H]);
            cmd(&mut p, 0x22, &[0xff]);
            for dot in [(0, 0), (1, 0)] {
                gx_window(&mut p, 0, 0, W as u16, H as u16);
                cmd(&mut p, 0x24, &image(W, H, &[dot]));
                cmd(&mut p, 0x20, &[]);
                wait(&mut p);
            }
            // the second image clears pixel 0 only if the RED RAM learnt that it is black
            assert_eq!((glass(&p, 0, 0), glass(&p, 1, 0), p.ping_pong), (if ping_pong { 0xff } else { 0 }, 0, ping_pong));
        }
    }

    #[test]
    fn ram_options_bypass_and_invert() {
        let mut p = panel();
        gx_window(&mut p, 0, 0, W as u16, H as u16);
        cmd(&mut p, 0x24, &image(W, H, &[(1, 0)]));
        cmd(&mut p, 0x21, &[0x08, 0x00]);
        cmd(&mut p, 0x22, &[0xf7]); cmd(&mut p, 0x20, &[]); wait(&mut p);
        assert_eq!((glass(&p, 0, 0), glass(&p, 1, 0)), (0, 0xff), "BW inverted");
        cmd(&mut p, 0x21, &[0x04, 0x00]);
        cmd(&mut p, 0x20, &[]); wait(&mut p);
        assert_eq!(p.black_pixels(), W * H, "BW bypassed as 0");
    }

    #[test]
    fn power_stages_busy_without_touching_the_glass() {
        let mut p = panel();
        cmd(&mut p, 0x22, &[0xe0]); cmd(&mut p, 0x20, &[]);
        assert_eq!(p.next_deadline(), Some(POWER_ON_US));
        wait(&mut p);
        cmd(&mut p, 0x22, &[0x83]); cmd(&mut p, 0x20, &[]);
        assert_eq!(p.next_deadline(), Some(POWER_ON_US + POWER_OFF_US));
        wait(&mut p);
        assert_eq!((p.version(), p.black_pixels()), (0, 0));
    }

    #[test]
    fn deep_sleep_holds_busy_until_the_reset_line() {
        let mut p = panel();
        gx_window(&mut p, 0, 0, W as u16, H as u16);
        cmd(&mut p, 0x24, &image(W, H, &[(9, 9)]));
        cmd(&mut p, 0x11, &[0x07]);
        cmd(&mut p, 0x10, &[0x01]);
        assert!(p.busy() && p.sleeping && p.next_deadline().is_none());
        cmd(&mut p, 0x11, &[0x00]);
        assert_eq!((p.entry, p.bytes_asleep), (0x07, 2), "the interface is off in deep sleep");
        p.advance_to(50);
        p.hw_reset();
        assert_eq!(p.take_edges().last(), Some(&BoardEdge { cycle: 50, pin: 3, level: false }));
        assert!(!p.busy() && !p.sleeping);
        assert_eq!((p.entry, p.xe, p.ye, p.update2), (0x03, 0x3bf, 0x2a7, 0xff), "registers back to their defaults");
        assert_eq!(p.bw[(H - 1 - 9) * STRIDE + 1], 0b1011_1111, "RAM kept");
    }

    #[test]
    fn reset_during_a_refresh_lands_it_at_once() {
        let mut p = panel();
        cmd(&mut p, 0x24, &[0x00]);
        cmd(&mut p, 0x22, &[0xf7]); cmd(&mut p, 0x20, &[]);
        assert!(p.busy());
        p.hw_reset();
        assert_eq!((p.busy(), p.version(), p.next_deadline()), (false, 1, None));
        assert_eq!(glass(&p, 0, H - 1), 0);
    }

    #[test]
    fn bytes_while_busy_are_counted_and_still_taken() {
        let mut p = panel();
        cmd(&mut p, 0x12, &[]);
        cmd(&mut p, 0x11, &[0x01]);
        assert_eq!((p.entry, p.bytes_while_busy), (0x01, 2));
    }
}
