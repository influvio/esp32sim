//! The board microphone, fed from the host: a test tone, an audio clip or live samples from the
//! page. The machine converts them to the rate the firmware's I2S receiver runs at and queues
//! them on that controller's per-controller input (`SocBus::i2s_input`), a short way ahead of
//! the receiver (`Machine::feed_mic`). A tone and a clip advance with the frames the receiver
//! takes, so a run is reproducible; live input is real time and drops what the emulation
//! cannot keep up with.
//!
//! This sits at the PCM boundary like the rest of I2S RX: no codec ADC, gain or filter.
use std::collections::VecDeque;
use std::sync::Arc;

pub const MIN_RATE: u32 = 8000;
pub const MAX_RATE: u32 = 96000;
/// Longest clip, in seconds of its own rate.
pub const MAX_CLIP_SECONDS: usize = 120;
/// Live input older than this is dropped, so a slow emulation does not build up delay.
pub const LIVE_LATENCY_MS: usize = 250;
/// Live input starts, and restarts after running dry, once this much has arrived: a jitter
/// buffer, so uneven delivery does not chop the stream into gaps.
pub const LIVE_PREFILL_MS: usize = 80;
/// How far ahead of the receiver the queue is kept.
pub const LOOKAHEAD_MS: u64 = 60;

#[derive(Clone)]
pub enum MicSource {
    Tone { hz: f64, amplitude: f64, phase: f64 },
    /// `pos` is the position in source frames; `looped` restarts at the end, otherwise silence
    Clip { rate: u32, frames: Arc<Vec<[i16; 2]>>, pos: f64, looped: bool },
    /// rate 0 until the first chunk; `primed` once `LIVE_PREFILL_MS` has arrived
    Live { rate: u32, frames: VecDeque<[i16; 2]>, pos: f64, primed: bool },
}

#[derive(Clone)]
pub struct Mic {
    pub source: MicSource,
    /// frames queued on the receiver, live frames dropped for latency
    pub fed: u64, pub dropped: u64,
}

impl std::fmt::Debug for Mic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { f.write_str(&self.describe()) }
}

fn check_rate(rate: u32) -> Result<(), String> {
    if (MIN_RATE..=MAX_RATE).contains(&rate) { Ok(()) } else { Err(format!("sample rate {rate} Hz is outside {MIN_RATE}..{MAX_RATE}")) }
}

fn lerp(a: [i16; 2], b: [i16; 2], t: f64) -> [i16; 2] {
    [0, 1].map(|c| (a[c] as f64 + (b[c] as f64 - a[c] as f64) * t).round() as i16)
}

impl Mic {
    fn new(source: MicSource) -> Self { Mic { source, fed: 0, dropped: 0 } }
    pub fn tone(hz: f64, amplitude: f64) -> Result<Self, String> {
        if !hz.is_finite() || !(0.0..=f64::from(MAX_RATE) / 2.0).contains(&hz) { return Err(format!("tone frequency must be 0..{} Hz", MAX_RATE / 2)); }
        if !amplitude.is_finite() || !(0.0..=1.0).contains(&amplitude) { return Err("tone amplitude must be 0..1".into()); }
        Ok(Self::new(MicSource::Tone { hz, amplitude, phase: 0.0 }))
    }
    pub fn clip(rate: u32, frames: Vec<[i16; 2]>, looped: bool) -> Result<Self, String> {
        check_rate(rate)?;
        if frames.is_empty() { return Err("the clip has no samples".into()); }
        if frames.len() > rate as usize * MAX_CLIP_SECONDS { return Err(format!("the clip is longer than {MAX_CLIP_SECONDS} s")); }
        Ok(Self::new(MicSource::Clip { rate, frames: Arc::new(frames), pos: 0.0, looped }))
    }
    pub fn live() -> Self { Self::new(MicSource::Live { rate: 0, frames: VecDeque::new(), pos: 0.0, primed: false }) }
    pub fn is_live(&self) -> bool { matches!(self.source, MicSource::Live { .. }) }

    /// Append live frames at `rate`. A new rate restarts the queue; the oldest frames beyond
    /// `LIVE_LATENCY_MS` are dropped. Ignored unless the source is live.
    pub fn push_live(&mut self, rate: u32, input: &[[i16; 2]]) -> Result<(), String> {
        check_rate(rate)?;
        let MicSource::Live { rate: r, frames, pos, primed } = &mut self.source else { return Ok(()) };
        if *r != rate { *r = rate; frames.clear(); *pos = 0.0; *primed = false; }
        frames.extend(input.iter().copied());
        let excess = frames.len().saturating_sub(rate as usize * LIVE_LATENCY_MS / 1000);
        if excess > 0 { frames.drain(..excess); self.dropped += excess as u64; }
        Ok(())
    }

    /// Up to `n` frames for a receiver running at `rate` (linear interpolation from the source
    /// rate). Fewer when a clip has ended or live input has not arrived yet.
    pub fn produce(&mut self, rate: u32, n: usize) -> Vec<[i16; 2]> {
        let mut out = Vec::with_capacity(n);
        if rate == 0 { return out; }
        match &mut self.source {
            MicSource::Tone { hz, amplitude, phase } => {
                for _ in 0..n {
                    out.push([(phase.sin() * *amplitude * 32767.0).round() as i16; 2]);
                    *phase = (*phase + std::f64::consts::TAU * *hz / f64::from(rate)) % std::f64::consts::TAU;
                }
            }
            MicSource::Clip { rate: src, frames, pos, looped } => {
                let step = f64::from(*src) / f64::from(rate);
                let len = frames.len();
                while out.len() < n {
                    if *pos >= len as f64 { if *looped { *pos -= len as f64; } else { break; } }
                    let i = *pos as usize;
                    let next = if i + 1 < len { frames[i + 1] } else if *looped { frames[0] } else { frames[i] };
                    out.push(lerp(frames[i], next, *pos - i as f64));
                    *pos += step;
                }
            }
            MicSource::Live { rate: src, frames, pos, primed } => {
                if *src == 0 { return out; }
                if !*primed && frames.len() < *src as usize * LIVE_PREFILL_MS / 1000 { return out; }
                *primed = true;
                let step = f64::from(*src) / f64::from(rate);
                while out.len() < n {
                    let i = *pos as usize;
                    if i + 1 >= frames.len() { *primed = false; break; }
                    out.push(lerp(frames[i], frames[i + 1], *pos - i as f64));
                    *pos += step;
                }
                let used = (*pos as usize).min(frames.len());
                frames.drain(..used);
                *pos -= used as f64;
            }
        }
        out
    }

    pub fn describe(&self) -> String {
        match &self.source {
            MicSource::Tone { hz, amplitude, .. } => format!("tone {hz} Hz at {amplitude}"),
            MicSource::Clip { rate, frames, looped, .. } => format!("clip {:.2} s at {rate} Hz{}", frames.len() as f64 / f64::from(*rate), if *looped { ", looped" } else { "" }),
            MicSource::Live { rate, .. } => if *rate == 0 { "live, waiting for samples".into() } else { format!("live at {rate} Hz") },
        }
    }
    pub fn report(&self) -> String { format!("[emu] mic: {}; {} frames queued on the receiver, {} live frames dropped", self.describe(), self.fed, self.dropped) }

    /// `tone HZ [AMPLITUDE]`, `file PATH [loop]`, `off` (the script's `mic` action).
    pub fn from_words(words: &[&str]) -> Result<Option<Mic>, String> {
        match words {
            ["off"] => Ok(None),
            ["tone", hz, rest @ ..] if rest.len() <= 1 => {
                let hz = hz.parse().map_err(|_| format!("bad frequency {hz}"))?;
                let amplitude = rest.first().map_or(Ok(0.5), |a| a.parse().map_err(|_| format!("bad amplitude {a}")))?;
                Mic::tone(hz, amplitude).map(Some)
            }
            ["file", path, rest @ ..] if rest.is_empty() || rest == ["loop"] => {
                let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
                let (rate, frames) = parse_wav(&bytes).map_err(|e| format!("{path}: {e}"))?;
                Mic::clip(rate, frames, !rest.is_empty()).map(Some)
            }
            _ => Err("expected `tone HZ [AMPLITUDE]`, `file PATH.wav [loop]` or `off`".into()),
        }
    }
}

/// Little-endian PCM16 frames of `channels` (1 or 2) as stereo frames; mono is duplicated.
pub fn pcm16_frames(bytes: &[u8], channels: usize) -> Result<Vec<[i16; 2]>, String> {
    if !(1..=2).contains(&channels) { return Err("1 or 2 channels".into()); }
    if !bytes.len().is_multiple_of(2 * channels) { return Err("PCM length is not a whole number of frames".into()); }
    Ok(bytes.chunks_exact(2 * channels).map(|f| {
        let l = i16::from_le_bytes([f[0], f[1]]);
        [l, if channels == 2 { i16::from_le_bytes([f[2], f[3]]) } else { l }]
    }).collect())
}

/// A RIFF WAVE file: integer PCM of 8, 16, 24 or 32 bits or 32-bit float, any channel count
/// (the first two are kept), 8..96 kHz. Returns the rate and stereo PCM16 frames.
pub fn parse_wav(b: &[u8]) -> Result<(u32, Vec<[i16; 2]>), String> {
    if b.len() < 12 || &b[0..4] != b"RIFF" || &b[8..12] != b"WAVE" { return Err("not a RIFF WAVE file".into()); }
    let u16_at = |o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
    let u32_at = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
    let (mut fmt, mut data) = (None, None);
    let mut o = 12;
    while o + 8 <= b.len() {
        let len = u32_at(o + 4) as usize;
        let body = o + 8..(o + 8).saturating_add(len).min(b.len());
        match &b[o..o + 4] { b"fmt " => fmt = Some(body.clone()), b"data" => data = Some(body.clone()), _ => {} }
        o = body.end + (len & 1);
    }
    let fmt = fmt.filter(|f| f.len() >= 16).ok_or("no fmt chunk")?;
    let data = data.ok_or("no data chunk")?;
    let mut tag = u16_at(fmt.start);
    if tag == 0xfffe && fmt.len() >= 26 { tag = u16_at(fmt.start + 24); }   // WAVE_FORMAT_EXTENSIBLE: the subformat's tag
    let channels = u16_at(fmt.start + 2) as usize;
    let rate = u32_at(fmt.start + 4);
    let bits = u16_at(fmt.start + 14) as usize;
    check_rate(rate)?;
    if channels == 0 { return Err("no channels".into()); }
    let width = match (tag, bits) { (1, 8 | 16 | 24 | 32) | (3, 32) => bits / 8, _ => return Err(format!("unsupported sample format (tag {tag}, {bits} bits): use PCM or 32-bit float")) };
    let frame = width * channels;
    let n = data.len() / frame;
    if n > rate as usize * MAX_CLIP_SECONDS { return Err(format!("longer than {MAX_CLIP_SECONDS} s")); }
    let sample = |p: usize| -> i16 {
        let s = &b[p..p + width];
        match (tag, width) {
            (3, _) => (f32::from_le_bytes([s[0], s[1], s[2], s[3]]).clamp(-1.0, 1.0) * 32767.0).round() as i16,
            (_, 1) => ((s[0] as i16) - 128) << 8,
            (_, 2) => i16::from_le_bytes([s[0], s[1]]),
            (_, 3) => i16::from_le_bytes([s[1], s[2]]),
            _ => i16::from_le_bytes([s[2], s[3]]),
        }
    };
    Ok((rate, (0..n).map(|i| {
        let p = data.start + i * frame;
        let l = sample(p);
        [l, if channels > 1 { sample(p + width) } else { l }]
    }).collect()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wav(tag: u16, channels: u16, rate: u32, bits: u16, data: &[u8]) -> Vec<u8> {
        let mut v = b"RIFF\0\0\0\0WAVE".to_vec();
        v.extend_from_slice(b"LIST\x03\0\0\0abc\0");                                      // odd chunk, padded
        v.extend_from_slice(b"fmt \x10\0\0\0");
        for x in [tag, channels] { v.extend_from_slice(&x.to_le_bytes()); }
        v.extend_from_slice(&rate.to_le_bytes());
        v.extend_from_slice(&(rate * u32::from(channels * bits / 8)).to_le_bytes());
        for x in [channels * bits / 8, bits] { v.extend_from_slice(&x.to_le_bytes()); }
        v.extend_from_slice(b"data"); v.extend_from_slice(&(data.len() as u32).to_le_bytes()); v.extend_from_slice(data);
        v
    }

    #[test]
    fn wav_formats() {
        assert_eq!(parse_wav(&wav(1, 1, 16000, 16, &[1, 0, 0xff, 0xff])).unwrap(), (16000, vec![[1, 1], [-1, -1]]));
        assert_eq!(parse_wav(&wav(1, 2, 8000, 8, &[255, 0])).unwrap().1, vec![[127 << 8, -128 << 8]]);
        assert_eq!(parse_wav(&wav(1, 1, 8000, 24, &[0x99, 0x34, 0x12])).unwrap().1, vec![[0x1234; 2]]);
        assert_eq!(parse_wav(&wav(3, 1, 48000, 32, &(-2.0f32).to_le_bytes())).unwrap().1, vec![[-32767; 2]]);
        assert_eq!(parse_wav(&wav(1, 3, 8000, 16, &[1, 0, 2, 0, 3, 0])).unwrap().1, vec![[1, 2]], "extra channels dropped");
        assert!(parse_wav(&wav(1, 1, 4000, 16, &[0, 0])).unwrap_err().contains("4000"));
        assert!(parse_wav(&wav(2, 1, 8000, 4, &[0, 0])).unwrap_err().contains("unsupported"));
        assert!(parse_wav(b"RIFF\0\0\0\0WAVE").is_err() && parse_wav(b"junk").is_err());
        let mut truncated = wav(1, 1, 8000, 16, &[1, 0, 2, 0]); truncated.truncate(truncated.len() - 1);
        assert_eq!(parse_wav(&truncated).unwrap().1, vec![[1, 1]], "a cut-off data chunk keeps its whole frames");
    }

    #[test]
    fn clip_resamples_and_ends_or_loops() {
        let mut m = Mic::clip(8000, vec![[0, 0], [100, -100]], false).unwrap();
        assert_eq!(m.produce(16000, 10), vec![[0, 0], [50, -50], [100, -100], [100, -100]]);
        assert!(m.produce(16000, 10).is_empty(), "silence after the end");
        let mut m = Mic::clip(8000, vec![[0, 0], [100, 100]], true).unwrap();
        assert_eq!(m.produce(8000, 5), vec![[0, 0], [100, 100], [0, 0], [100, 100], [0, 0]]);
    }

    #[test]
    fn live_waits_for_input_and_caps_latency() {
        let mut m = Mic::live();
        assert!(m.produce(16000, 4).is_empty());
        let prefill = 16000 * LIVE_PREFILL_MS / 1000;                                    // 1280 frames
        m.push_live(16000, &vec![[0, 0]; prefill - 3]).unwrap();
        m.push_live(16000, &[[1, 1], [2, 2]]).unwrap();
        assert!(m.produce(16000, 10).is_empty(), "waits for the jitter buffer");
        m.push_live(16000, &[[3, 3]]).unwrap();
        assert_eq!(m.produce(16000, prefill), [vec![[0, 0]; prefill - 3], vec![[1, 1], [2, 2]]].concat(), "the last frame waits for its successor");
        m.push_live(16000, &[[4, 4]]).unwrap();
        assert!(m.produce(16000, 10).is_empty(), "ran dry: refills first");
        m.push_live(16000, &vec![[5, 5]; prefill]).unwrap();
        assert_eq!(m.produce(16000, 2), vec![[3, 3], [4, 4]]);
        m.push_live(8000, &vec![[7, 7]; 3000]).unwrap();
        assert_eq!((m.dropped, m.produce(8000, 1)), (1000, vec![[7, 7]]), "a new rate restarts; 250 ms kept");
        assert!(m.push_live(100, &[]).is_err());
    }

    #[test]
    fn tone_and_words() {
        let mut m = Mic::tone(4000.0, 1.0).unwrap();
        assert_eq!(m.produce(16000, 4), vec![[0; 2], [32767; 2], [0; 2], [-32767; 2]]);
        assert!(Mic::from_words(&["off"]).unwrap().is_none());
        assert_eq!(Mic::from_words(&["tone", "440"]).unwrap().unwrap().describe(), "tone 440 Hz at 0.5");
        assert!(Mic::from_words(&["tone", "x"]).is_err() && Mic::from_words(&["tone", "1", "2"]).is_err() && Mic::from_words(&["file"]).is_err());
        assert_eq!(pcm16_frames(&[1, 0, 2, 0], 2).unwrap(), vec![[1, 2]]);
        assert!(pcm16_frames(&[1, 0, 2], 1).is_err() && pcm16_frames(&[], 3).is_err());
    }
}
