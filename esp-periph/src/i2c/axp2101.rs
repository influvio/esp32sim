//! X-Powers AXP2101 power-on key and interrupt controller (Waveshare's schematics call the chip
//! TG28). Source: TG28 datasheet V1.0 (X-Powers, 2024-05-11), sections 6.5.4 (the power on-off
//! key, OFFLEVEL) and 6.12.1 (IRQ); registers 0x10, 0x21, 0x22, 0x27, 0x40-0x42 and 0x48-0x4A.
//! The other registers are storage, with the chip ID 0x4a at 0x03 that XPowersLib checks. No
//! regulator, charger, ADC or fuel gauge is modelled, and a power-off is only recorded: the SoC
//! the PMIC supplies keeps running.
//!
//! The datasheet names the key's four events but not their timing. Taken from the AXP2101
//! convention: the negative edge at the press, the long press once it has been held for IRQLEVEL,
//! the short press at a release before that, the positive edge at every release.
use super::I2cDevice;
use std::sync::{Arc, Mutex};

/// REG 49H (IRQ status 1) and REG 41H (its enables): the PWRON key's events.
pub const PKEY_SHORT: u8 = 1 << 3;
pub const PKEY_LONG: u8 = 1 << 2;
pub const PKEY_NEGATIVE: u8 = 1 << 1;
pub const PKEY_POSITIVE: u8 = 1 << 0;

/// Register file and key timing, shared by the I2C device and the board that wires the key.
pub struct Axp2101State {
    pub regs: [u8; 256],
    cycles_per_ms: u64,
    /// the cycle the key went down, while it is held
    pressed_at: Option<u64>,
    long_sent: bool,
    off_sent: bool,
    /// why the PMIC would have switched its outputs off (not modelled beyond this record)
    pub power_off: Option<&'static str>,
    pub key_presses: u64,
}

impl Axp2101State {
    pub fn new(cpu_hz: u64) -> Self {
        let mut regs = [0u8; 256];
        // Defaults (datasheet section 6.13.2): REG 10H bits 5:4 = 1; REG 22H bit 2 = 1, bit 1
        // (PWRON > OFFLEVEL powers off) is an eFuse value, taken as enabled; REG 27H IRQLEVEL 1.5 s,
        // OFFLEVEL 6 s, ONLEVEL an eFuse value, taken as 128 ms; IRQ enables 0xff / 0xfc / 0x5f.
        for (r, v) in [(0x03, 0x4a), (0x10, 0x30), (0x22, 0x06), (0x27, 0x14), (0x40, 0xff), (0x41, 0xfc), (0x42, 0x5f)] { regs[r] = v; }
        Axp2101State { regs, cycles_per_ms: (cpu_hz / 1000).max(1), pressed_at: None, long_sent: false, off_sent: false, power_off: None, key_presses: 0 }
    }
    pub fn key_down(&self) -> bool { self.pressed_at.is_some() }
    /// The IRQ pin is pulled low while an event's status bit is set (section 6.12.1).
    pub fn irq_low(&self) -> bool { (self.regs[0x48] | self.regs[0x49] | self.regs[0x4a]) != 0 }
    /// A status bit is set only if its enable bit is.
    fn raise(&mut self, bit: u8) { self.regs[0x49] |= bit & self.regs[0x41]; }
    fn irq_level(&self) -> u64 { [1000, 1500, 2000, 2500][((self.regs[0x27] >> 4) & 3) as usize] * self.cycles_per_ms }
    fn off_level(&self) -> u64 { [4000, 6000, 8000, 10000][((self.regs[0x27] >> 2) & 3) as usize] * self.cycles_per_ms }
    /// The PWRON key pressed (`down`) or released at `cycle`.
    pub fn key(&mut self, cycle: u64, down: bool) {
        self.advance_to(cycle);
        match (down, self.pressed_at) {
            (true, None) => {
                self.pressed_at = Some(cycle); self.long_sent = false; self.off_sent = false; self.key_presses += 1;
                self.raise(PKEY_NEGATIVE);
            }
            (false, Some(_)) => {
                if !self.long_sent { self.raise(PKEY_SHORT); }
                self.raise(PKEY_POSITIVE);
                self.pressed_at = None;
            }
            _ => {}
        }
    }
    /// The next long-press or OFFLEVEL moment of a held key.
    pub fn next_deadline(&self) -> Option<u64> {
        let at = self.pressed_at?;
        if !self.long_sent { Some(at.saturating_add(self.irq_level())) }
        else if !self.off_sent && self.regs[0x22] & 2 != 0 { Some(at.saturating_add(self.off_level())) }
        else { None }
    }
    pub fn advance_to(&mut self, cycle: u64) {
        while let Some(due) = self.next_deadline() {
            if due > cycle { break; }
            if !self.long_sent { self.long_sent = true; self.raise(PKEY_LONG); }
            else { self.off_sent = true; self.regs[0x21] |= 1; self.power_off.get_or_insert("PWRON held for OFFLEVEL"); }
        }
    }
    /// A register write from the host.
    pub fn write(&mut self, reg: u8, value: u8) {
        match reg {
            0x00 | 0x01 | 0x20 | 0x21 => {}                                   // read-only status
            0x48..=0x4a => self.regs[reg as usize] &= !value,                // RW1C
            0x10 => {
                // bits 1 (restart) and 0 (soft power-off) clear themselves
                if value & 1 != 0 { self.regs[0x21] |= 2; self.power_off.get_or_insert("REG 10H soft power-off"); }
                if value & 2 != 0 { self.power_off.get_or_insert("REG 10H restart"); }
                self.regs[0x10] = value & !3;
            }
            _ => self.regs[reg as usize] = value,
        }
    }
}

/// The AXP2101 on an I2C bus: an 8-bit register pointer, then data that auto-increments it.
pub struct Axp2101 { state: Arc<Mutex<Axp2101State>>, ptr: u8, first: bool }

impl Axp2101 {
    pub const ADDRESS: u8 = 0x34;
    pub fn new(state: Arc<Mutex<Axp2101State>>) -> Self { Axp2101 { state, ptr: 0, first: true } }
}

impl I2cDevice for Axp2101 {
    fn start(&mut self, read: bool) -> bool { if !read { self.first = true; } true }
    fn write(&mut self, b: u8) -> bool {
        if self.first { self.ptr = b; self.first = false; }
        else { self.state.lock().expect("AXP2101 state mutex poisoned").write(self.ptr, b); self.ptr = self.ptr.wrapping_add(1); }
        true
    }
    fn read(&mut self) -> u8 {
        let v = self.state.lock().expect("AXP2101 state mutex poisoned").regs[self.ptr as usize];
        self.ptr = self.ptr.wrapping_add(1);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 240_000;

    #[test]
    fn short_press_edges_and_clearing() {
        let mut p = Axp2101State::new(240_000_000);
        p.write(0x41, 0xff);                                           // both edges enabled too
        p.key(10, true);
        assert_eq!((p.regs[0x49], p.irq_low(), p.key_down()), (PKEY_NEGATIVE, true, true));
        p.write(0x49, 0xff);
        assert!(!p.irq_low());
        p.key(200 * MS, false);
        assert_eq!(p.regs[0x49], PKEY_SHORT | PKEY_POSITIVE);
        assert_eq!(p.next_deadline(), None);
    }

    #[test]
    fn long_press_and_offlevel_follow_reg_27() {
        let mut p = Axp2101State::new(240_000_000);
        assert_eq!(p.regs[0x41] & (PKEY_NEGATIVE | PKEY_POSITIVE), 0, "edge interrupts are off by default");
        p.write(0x27, 0x01);                                           // IRQLEVEL 1 s, OFFLEVEL 4 s, as Waveshare sets it
        p.key(0, true);
        assert_eq!((p.regs[0x49], p.next_deadline()), (0, Some(1000 * MS)));
        p.advance_to(1000 * MS - 1);
        assert_eq!(p.regs[0x49], 0);
        p.advance_to(1000 * MS);
        assert_eq!((p.regs[0x49], p.next_deadline()), (PKEY_LONG, Some(4000 * MS)));
        p.advance_to(4000 * MS);
        assert_eq!((p.power_off, p.regs[0x21]), (Some("PWRON held for OFFLEVEL"), 1));
        p.key(5000 * MS, false);
        assert_eq!(p.regs[0x49], PKEY_LONG, "no short press after a long one");
    }

    #[test]
    fn device_reads_and_writes_through_the_pointer() {
        let state = Arc::new(Mutex::new(Axp2101State::new(240_000_000)));
        let mut d = Axp2101::new(state.clone());
        d.start(false); d.write(0x03);
        d.start(true);
        assert_eq!(d.read(), 0x4a);
        state.lock().unwrap().key(0, true);
        state.lock().unwrap().key(1, false);
        d.start(false); d.write(0x48); d.start(true);
        assert_eq!([d.read(), d.read(), d.read()], [0, PKEY_SHORT, 0]);
        d.start(false); d.write(0x48); d.write(0xff); d.write(0xff); d.write(0xff);
        assert!(!state.lock().unwrap().irq_low());
        d.start(false); d.write(0x10); d.write(0x31);
        let s = state.lock().unwrap();
        assert_eq!((s.regs[0x10], s.power_off), (0x30, Some("REG 10H soft power-off")));
    }
}
