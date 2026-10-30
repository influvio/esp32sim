//! The board microphone: the host source (`crate::mic`) kept a short way ahead of the I2S
//! receiver the board's codec reaches. With a page the input poll tops it up; without one a
//! recurring script action does, every `FEED_INTERVAL_MS` of emulated time.
use super::*;
use crate::mic::{Mic, LOOKAHEAD_MS, MAX_RATE, MIN_RATE};

pub(super) const FEED_INTERVAL_MS: u64 = 20;

impl<S: Soc> Machine<S> {
    /// Replace the board microphone's source (None: silence). Clears what the previous source
    /// had queued on the board's receivers, including an `--i2s-tone` there.
    pub fn set_mic(&mut self, mic: Option<Mic>) -> Result<(), String> {
        let ports = self.bus.board_ref().mic_ports();
        if ports.is_empty() { return Err(format!("board {} has no microphone", self.bus.board_ref().name())); }
        for &port in ports { if let Some(input) = self.bus.i2s_input(port) { input.clear(); } }
        self.mic = mic.map(Box::new);
        self.schedule_mic_feed(0);
        self.feed_mic();
        Ok(())
    }

    /// Queue frames on the first of the board's receivers that runs, up to `LOOKAHEAD_MS` ahead.
    /// A rate outside 8..96 kHz (a receiver clock the firmware has not finished setting up) waits.
    pub(super) fn feed_mic(&mut self) {
        let Some(mic) = self.mic.as_mut() else { return };
        for &port in self.bus.board_ref().mic_ports() {
            let Some(rate) = self.bus.i2s_rx_rate(port) else { continue };
            if !(MIN_RATE..=MAX_RATE).contains(&rate) { return; }
            let Some(input) = self.bus.i2s_input(port) else { continue };
            let target = (u64::from(rate) * LOOKAHEAD_MS / 1000) as usize;
            let queued = input.queued_frames();
            if queued < target {
                let frames = mic.produce(rate, target - queued);
                mic.fed += input.push(&frames) as u64;
            }
            return;
        }
    }

    /// Without a page, keep one `MicFeed` action pending `delay_ms` from now while a source is set.
    pub(super) fn schedule_mic_feed(&mut self, delay_ms: u64) {
        if self.mic.is_none() || self.web.is_some() { return; }
        let pos = self.script.pos;
        let events = &mut self.script.events;
        let mut i = pos;
        while i < events.len() { if matches!(events[i].1, ScriptAction::MicFeed) { events.remove(i); } else { i += 1; } }
        let at = self.bus.cycles() + delay_ms * S::CPU_HZ / 1000;
        let slot = pos + events[pos..].partition_point(|e| e.0 <= at);
        events.insert(slot, (at, ScriptAction::MicFeed));
    }
}
