//! Host messages are untrusted even when a browser normally sends small inputs.
use esp_soc::{board::BoardModel, picture::Picture, web::WebServer, ScriptAction, Stop};
use std::sync::{Arc, Mutex};

struct InputBoard(Arc<Mutex<Vec<Picture>>>);

impl BoardModel for InputBoard {
    fn name(&self) -> &'static str { "input-test" }
    fn encoder(&self) -> Option<(u8, u8)> { Some((1, 2)) }
    fn set_camera_picture(&mut self, picture: Picture) -> Result<(), &'static str> { self.0.lock().unwrap().push(picture); Ok(()) }
}

fn fixture() -> (esp32s3::Machine, WebServer, Arc<Mutex<Vec<Picture>>>) {
    let mut machine = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    let pictures = Arc::new(Mutex::new(Vec::new()));
    machine.bus.board = Box::new(InputBoard(pictures.clone()));
    let web = WebServer::queued();
    machine.web = Some(web.clone());
    (machine, web, pictures)
}

fn camera_message(width: u16, height: u16, rgba: &[u8]) -> Vec<u8> {
    let mut message = vec![3];
    message.extend_from_slice(&width.to_le_bytes());
    message.extend_from_slice(&height.to_le_bytes());
    message.extend_from_slice(rgba);
    message
}

#[test]
fn camera_input_rejects_invalid_sizes_and_preserves_valid_rgb() {
    let (mut machine, web, pictures) = fixture();
    for message in [
        vec![],
        vec![3],
        camera_message(0, 1, &[1, 2, 3, 4]),
        camera_message(1, 0, &[1, 2, 3, 4]),
        camera_message(2, 1, &[1, 2, 3, 4]),
        camera_message(32768, 32768, &[]),
        camera_message(u16::MAX, u16::MAX, &[]),
    ] {
        web.push_incoming_bin(message);
    }
    web.push_incoming_bin(camera_message(2, 1, &[10, 20, 30, 0, 40, 50, 60, 255]));
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    let pictures = pictures.lock().unwrap();
    assert_eq!(pictures.len(), 1);
    assert_eq!((pictures[0].w, pictures[0].h), (2, 1));
    assert_eq!(pictures[0].rgb, [10, 20, 30, 40, 50, 60]);
    assert_eq!(machine.bus.cycles, 0);
    assert!(web.poll_incoming_bin().is_empty());
}

#[test]
fn knob_extreme_deltas_have_bounded_work_and_keep_their_direction() {
    for delta in [i32::MIN, i32::MAX] {
        let (mut machine, web, _) = fixture();
        web.push_incoming(format!(r#"{{"t":"knob","d":{delta}}}"#));
        assert!(matches!(machine.run(0), Stop::MaxInsns));
        assert_eq!(machine.script.events.len(), 64 * 4);
        let first_pin = if delta > 0 { 1 } else { 2 };
        assert!(matches!(machine.script.events[0].1, ScriptAction::Gpio(pin, false) if pin == first_pin));
        assert_eq!(machine.bus.cycles, 0);
        assert!(web.poll_incoming().is_empty());
    }
}

#[test]
fn ordinary_knob_deltas_preserve_edges_spacing_and_queue_order() {
    let (mut machine, web, _) = fixture();
    for delta in [2, 0, -1] {
        web.push_incoming(format!(r#"{{"t":"knob","d":{delta}}}"#));
    }
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    let step = 240_000_000 / 500;
    let expected = [
        (1, 1, false), (2, 2, false), (3, 1, true), (4, 2, true),
        (9, 1, false), (10, 2, false), (11, 1, true), (12, 2, true),
        (17, 2, false), (18, 1, false), (19, 2, true), (20, 1, true),
    ];
    assert_eq!(machine.script.events.len(), expected.len());
    for ((cycle, action), (phase, expected_pin, expected_level)) in machine.script.events.iter().zip(expected) {
        assert_eq!(*cycle, phase * step);
        assert!(matches!(action, ScriptAction::Gpio(pin, level) if *pin == expected_pin && *level == expected_level));
    }
}

#[test]
fn camera_plain_input_channel_needs_no_web_server() {
    let (mut machine, _, pictures) = fixture();
    machine.web = None;
    machine.camera_input = Some(Arc::new(Mutex::new(Some(Picture { w: 1, h: 1, rgb: vec![1, 2, 3] }))));
    machine.run(0);
    assert_eq!(pictures.lock().unwrap()[0].rgb, [1, 2, 3]);
    assert!(machine.camera_input.as_ref().unwrap().lock().unwrap().is_none());
    assert!(machine.web.is_none());
}

/// The ePaper-3.97's PWR is a board key, not a GPIO button: the page's `{"t":"btn","key":"pwr"}`
/// and a script's `press pwr` both reach `key_at`, which raises PWR_OUT (GPIO 1).
#[test]
fn board_keys_from_the_page_and_scripts() {
    let mut machine = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    machine.bus.board = Box::new(esp32s3::board::WaveshareEpaper397::new());
    machine.bus.attach_board_devices();
    let web = WebServer::queued();
    machine.web = Some(web.clone());
    web.push_incoming(r#"{"t":"btn","key":"pwr","v":1}"#.to_string());
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    assert!(machine.bus.board.report().contains("axp2101: 1 PWR presses"));
    web.push_incoming(r#"{"t":"btn","key":"nope","v":1}"#.to_string());
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    machine.load_script("0.5 press pwr 200\n1 press boot\n").unwrap();
    assert!(matches!(machine.script.events[0].1, ScriptAction::Key(ref k, true) if k == "pwr"));
    assert!(matches!(machine.script.events[1].1, ScriptAction::Key(ref k, false) if k == "pwr"));
    assert!(matches!(machine.script.events[2].1, ScriptAction::Gpio(0, false)));
    assert!(machine.load_script("0.5 press nope\n").is_err());
}

/// I2S0 receiving 16-bit stereo at 8 kHz (the register values of esp-periph's RX tests).
fn epaper_receiving() -> esp32s3::Machine {
    let mut machine = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    machine.bus.board = Box::new(esp32s3::board::WaveshareEpaper397::new());
    machine.bus.attach_board_devices();
    let i2s = &mut machine.bus.periph.i2s0;
    i2s.write(0x30, (1 << 26) | (2 << 27) | 25);
    i2s.write(0x28, (24 << 7) | (15 << 13) | (15 << 18) | (15 << 24));
    i2s.write(0x50, (1 << 16) | 3);
    i2s.write(0x20, 4);
    machine
}

/// The microphone keeps 60 ms queued on the running receiver; without a page a script action
/// tops it up. `mic` needs a board with a microphone.
#[test]
fn mic_tone_feeds_the_running_receiver() {
    use esp_soc::SocBus;
    let mut machine = epaper_receiving();
    assert_eq!(machine.bus.i2s_rx_rate(0), Some(8000));
    assert_eq!(machine.bus.i2s_rx_rate(1), None);
    machine.load_script("0 mic tone 1000 0.5\n").unwrap();
    assert!(matches!(machine.script.events[0].1, ScriptAction::Mic(Some(_))));
    machine.set_mic(Some(esp_soc::mic::Mic::tone(1000.0, 0.5).unwrap())).unwrap();
    assert_eq!(machine.bus.i2s_input(0).unwrap().queued_frames(), 480);
    assert_eq!(machine.mic.as_ref().unwrap().fed, 480);
    assert_eq!(machine.script.events.iter().filter(|e| matches!(e.1, ScriptAction::MicFeed)).count(), 1);
    machine.set_mic(None).unwrap();
    assert_eq!(machine.bus.i2s_input(0).unwrap().queued_frames(), 0);
    let mut bare = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    assert!(bare.load_script("0 mic tone 440\n").unwrap_err().contains("no microphone"));
    assert!(bare.set_mic(None).is_err());
}

/// The page's microphone: a tone request, live chunks at 16 kHz resampled to the receiver's
/// 8 kHz, a clip, and malformed input answered with `mic-error`.
#[test]
fn mic_from_the_page() {
    let mut machine = epaper_receiving();
    let web = WebServer::queued();
    machine.web = Some(web.clone());
    let texts = |web: &WebServer| web.take_outbox().into_iter().filter(|m| m.0 == 1).map(|m| String::from_utf8(m.1).unwrap()).collect::<Vec<_>>();
    web.push_incoming(r#"{"t":"mic","src":"tone","hz":"440","amp":"0.25"}"#.to_string());
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    assert!(texts(&web).iter().any(|t| t.contains(r#""state":"tone 440 Hz at 0.25""#)));
    let mut live = vec![5, 0x80, 0x3e, 0, 0, 1];                                          // 16000 Hz, mono
    for i in 0..1600i16 { live.extend_from_slice(&i.to_le_bytes()); }
    web.push_incoming_bin(live);
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    assert!(texts(&web).iter().any(|t| t.contains("live at 16000 Hz")));
    use esp_soc::SocBus;
    assert_eq!(machine.bus.i2s_input(0).unwrap().queued_frames(), 480, "60 ms at 8 kHz from 100 ms at 16 kHz");
    let mut clip = vec![6, 0x40, 0x1f, 0, 0, 2, 1];                                       // 8000 Hz, stereo, looped
    clip.extend_from_slice(&[1, 0, 2, 0]);
    web.push_incoming_bin(clip);
    web.push_incoming_bin(vec![5, 0x40, 0x1f, 0, 0, 3, 0, 0, 0]);
    web.push_incoming_bin(vec![6, 0x10, 0, 0, 0, 1, 0, 1, 0]);
    assert!(matches!(machine.run(0), Stop::MaxInsns));
    let sent = texts(&web);
    assert!(sent.iter().any(|t| t.contains("clip 0.00 s at 8000 Hz, looped")));
    assert!(sent.iter().any(|t| t.contains("mic-error") && t.contains("channels")));
    assert!(sent.iter().any(|t| t.contains("mic-error") && t.contains("16 Hz")));
}
