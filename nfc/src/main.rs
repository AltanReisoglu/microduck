//! `nfcd` — owns the NFC reader, and pairs the gamepad a touched tag names.
//!
//! ## What it does
//!
//! Polls the reader five times a second. When a tag arrives, the first Bluetooth address in its
//! NDEF is taken as a pad's: if a pad is already connected nothing happens, otherwise `configd` is
//! asked to pair that one and `robotd` to quack once it has. The rule and why it is that rule are
//! in [`nfc::pairing`].
//!
//! ## Why its own daemon
//!
//! The reader is a device someone has to own, and nothing else here wants it: `configd` is in the
//! recovery path and must not wait on a serial port, and `padd` is unprivileged on purpose. So this
//! owns the port and is an ordinary client of the two sockets for everything else — the same
//! `pad.status`, `pad.pair` and `robot.sound` that `robotctl pad pair <mac>` and `robotctl quack`
//! send. It holds no Bluetooth access at all.
//!
//! ## No reader is the ordinary case
//!
//! Most robots have none fitted. A missing port is one line in the journal and a retry every few
//! seconds, so plugging a reader in is all it takes — no restart, nothing to enable.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use nfc::clrc663::Clrc663;
use nfc::pairing::{self, Outcome, Robot, Touches};
use nfc::serial::Serial;
use nfc::{Error, ndef, tag};

/// Between attempts to open a reader that is not there, doubling to the cap.
///
/// Absent is forever on most robots, so this settles at one quiet attempt every half minute; a
/// reader plugged in is found within that.
const RETRY_MIN: Duration = Duration::from_secs(2);
const RETRY_MAX: Duration = Duration::from_secs(30);

/// How long `pad.pair` may take to answer. `configd` looks for 15 s by default and then says so;
/// this only has to outlast that, so a wedged `configd` does not stop the reader for good.
const PAIR_ANSWER: Duration = Duration::from_secs(40);
const QUICK_ANSWER: Duration = Duration::from_secs(5);

#[derive(Parser, Debug)]
#[command(
    name = "nfcd",
    about = "NFC reader daemon: touch a tag, pair its gamepad",
    version
)]
struct Args {
    /// The reader's serial port.
    #[arg(long, default_value = "/dev/ttyACM0")]
    device: PathBuf,

    /// Polls a second. Detecting a tag alone costs ~35 ms, so 20 is the ceiling.
    #[arg(long, default_value_t = 5.0)]
    hz: f64,

    #[arg(long, default_value = proto::socket::CONFIG)]
    config_socket: PathBuf,

    #[arg(long, default_value = proto::socket::ROBOT)]
    robot_socket: PathBuf,
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    duck_ipc_proto::log_startup_identity!("nfcd");

    if !(args.hz > 0.0 && args.hz <= 20.0) {
        tracing::error!(hz = args.hz, "--hz must be above 0 and at most 20");
        return std::process::ExitCode::FAILURE;
    }
    let period = Duration::from_secs_f64(1.0 / args.hz);
    let mut robot = Sockets {
        config: args.config_socket.clone(),
        robot: args.robot_socket.clone(),
    };

    let mut backoff = RETRY_MIN;
    // The last reason there was no reader, so a reader that stays absent is one line, not one a
    // retry.
    let mut said: Option<String> = None;
    loop {
        match open(&args.device) {
            Ok(mut chip) => {
                tracing::warn!(device = %args.device.display(), hz = args.hz, "reader up — touch a tag");
                said = None;
                backoff = RETRY_MIN;
                let why = watch(&mut chip, period, &mut robot);
                tracing::warn!(device = %args.device.display(), error = %why, "reader lost");
            }
            Err(e) => {
                let why = e.to_string();
                if said.as_deref() != Some(why.as_str()) {
                    tracing::info!(device = %args.device.display(), error = %why, "no NFC reader; retrying");
                    said = Some(why);
                }
                sleep(backoff);
                backoff = (backoff * 2).min(RETRY_MAX);
            }
        }
    }
}

fn open(device: &Path) -> nfc::Result<Clrc663<Serial>> {
    let mut chip = Clrc663::new(Serial::open(device)?);
    chip.begin()?;
    Ok(chip)
}

/// Poll until the reader stops answering, acting on every touch. Returns why it stopped.
fn watch(chip: &mut Clrc663<Serial>, period: Duration, robot: &mut dyn Robot) -> Error {
    let mut touches = Touches::default();
    loop {
        let start = Instant::now();
        let uid = match tag::select(chip) {
            Ok(uid) => Some(uid),
            Err(e) if e.is_tag() => None,
            Err(e) => {
                // A desynchronised link comes back with a soft reset; an unplugged one does not,
                // and is the caller's to reopen.
                if matches!(e, Error::Desync(_)) && chip.begin().is_ok() {
                    tracing::info!(error = %e, "reader out of step; reset");
                    continue;
                }
                return e;
            }
        };

        if let Some(uid) = uid.as_deref()
            && touches.seen(Some(uid))
        {
            let id = hex(uid);
            match tag::read_ndef_area(chip).map(|data| ndef::parse(&data)) {
                Err(e) if !e.is_tag() => return e,
                Err(e) => {
                    tracing::info!(tag = %id, error = %e, "tag not read; trying again");
                    touches.unread();
                }
                Ok(Err(why)) => tracing::warn!(tag = %id, %why, "tag holds no readable NDEF"),
                Ok(Ok(records)) => match ndef::mac_in(&records) {
                    None => tracing::warn!(tag = %id, ?records, "tag carries no Bluetooth address"),
                    Some(mac) => {
                        // Off while the pairing blocks: no reason to power the antenna for the
                        // fifteen seconds nobody is reading it.
                        if let Err(e) = chip.field_off() {
                            return e;
                        }
                        tracing::warn!(tag = %id, %mac, "tag touched");
                        report(&mac, pairing::on_touch(robot, &mac));
                    }
                },
            }
        } else if uid.is_none() {
            touches.seen(None);
        }

        if let Some(rest) = period.checked_sub(start.elapsed()) {
            sleep(rest);
        }
    }
}

fn report(mac: &str, outcome: Outcome) {
    match outcome {
        Outcome::AlreadyConnected(pad) => {
            tracing::warn!(%mac, driving = %pad.mac, name = %pad.name, "a pad is already connected — nothing to do")
        }
        Outcome::Paired(pad) => tracing::warn!(mac = %pad.mac, name = %pad.name, "paired — quack"),
        Outcome::PairedSilently(pad, why) => {
            tracing::warn!(mac = %pad.mac, name = %pad.name, %why, "paired, but the robot would not quack")
        }
        Outcome::NotPaired(reason, detail) => tracing::warn!(
            %mac, ?reason, ?detail,
            "not paired — is the pad in pairing mode? Lift the tag and touch again to retry"
        ),
        Outcome::Failed(why) => tracing::error!(%mac, %why, "could not ask the robot"),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(":")
}

/// `configd` and `robotd`, a connection per call.
///
/// Fresh each time because touches are rare and either daemon may have restarted since the last
/// one: a held connection would be a dead one exactly when someone touches a tag.
struct Sockets {
    config: PathBuf,
    robot: PathBuf,
}

impl Robot for Sockets {
    fn pads(&mut self) -> Result<Vec<proto::Pad>, String> {
        let status: proto::PadStatusResult =
            call(&self.config, &proto::Call::PadStatus, QUICK_ANSWER)?;
        Ok(status.pads)
    }

    fn pair(&mut self, mac: &str) -> Result<proto::PadPairResult, String> {
        let params = proto::PadPairParams {
            mac: Some(mac.to_owned()),
            timeout_seconds: None,
        };
        call(&self.config, &proto::Call::PadPair(params), PAIR_ANSWER)
    }

    fn quack(&mut self) -> Result<(), String> {
        // The same sound `robotctl quack` plays: the mouth-trigger quack, in this robot's voice.
        let sound = proto::Call::RobotSound(proto::SoundParams {
            tag: proto::SoundTag::Chirp,
            hold: None,
        });
        let outcome: proto::IntentResult = call(&self.robot, &sound, QUICK_ANSWER)?;
        if outcome.accepted {
            Ok(())
        } else {
            Err(outcome.reason.unwrap_or_else(|| "refused".into()))
        }
    }
}

fn call<T: for<'de> serde::Deserialize<'de>>(
    socket: &Path,
    call: &proto::Call,
    answer_within: Duration,
) -> Result<T, String> {
    let fail = |e: std::io::Error| format!("{}: {e}", socket.display());
    let mut stream = UnixStream::connect(socket).map_err(fail)?;
    stream.set_read_timeout(Some(answer_within)).map_err(fail)?;
    let mut line = serde_json::to_vec(&proto::Request::call(proto::Id::Number(1), call))
        .map_err(|e| e.to_string())?;
    line.push(b'\n');
    stream.write_all(&line).map_err(fail)?;

    let mut answer = String::new();
    BufReader::new(stream)
        .read_line(&mut answer)
        .map_err(fail)?;
    let response: proto::Response =
        serde_json::from_str(&answer).map_err(|e| format!("unparsable answer: {e}"))?;
    if let Some(error) = response.error {
        return Err(error.message);
    }
    response
        .result_as()
        .map_err(|e| format!("unexpected answer: {e}"))
}
