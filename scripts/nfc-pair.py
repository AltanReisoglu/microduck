#!/usr/bin/env python3
"""Pair a gamepad by touching an NFC tag that carries its MAC address. A bench test, not a daemon.

The procedure: the reader is polled at 5 Hz. When a tag arrives, its NDEF is read and the first
Bluetooth address found in it is taken as the pad's. If a pad is already connected nothing
happens — the robot has a driver, and a tag brushed against it must not steal the bond. Otherwise
the pad at that address is paired, and the robot quacks once it is.

The pad has to be in pairing mode when the tag is touched (Sync on an Xbox pad, until the light
flashes fast): the tag says *which* pad, it does not wake one up.

For now the reader is a CLRC663 board on USB serial, driven by the `ntag663` library from the
winnie repo, which is Python — hence a script here rather than a daemon. Everything the robot
does is reached through `robotctl`, exactly as a person at a shell would, so this needs no access
of its own: `pad status`, `pad pair <mac>` and `quack`. `pad pair` stops `btd` while it pairs,
which is why this runs under sudo.

On the board:

    python3 -m venv ~/nfc && ~/nfc/bin/pip install ~/winnie/lib_winnie
    sudo ~/nfc/bin/python scripts/nfc-pair.py            # --port /dev/ttyACM0 by default

Writing a tag, from the laptop with the board plugged in:

    ntag write --text 98:B6:E9:28:06:09

A text or URI record both work, with `:` or `-` between the octets or none at all.

**One attempt per touch.** A tag left on the reader is handled once, on arrival; lift it and touch
again to retry. Otherwise a failed pairing would repeat every 200 ms for as long as the tag sat
there, each attempt holding discovery open for its whole window.
"""

import argparse
import json
import re
import subprocess
import sys
import time

# Twelve hex digits, in pairs, joined by `:` or `-` or by nothing. The pairs are captured separately
# so all three spellings normalise to the one `robotctl` and BlueZ print.
MAC = re.compile(
    r"(?<![0-9A-Fa-f])"
    r"([0-9A-Fa-f]{2})([:-]?)([0-9A-Fa-f]{2})\2([0-9A-Fa-f]{2})\2"
    r"([0-9A-Fa-f]{2})\2([0-9A-Fa-f]{2})\2([0-9A-Fa-f]{2})"
    r"(?![0-9A-Fa-f])"
)

# Polls in a row without the tag before it counts as lifted. One missed WUPA on a tag that never
# moved is ordinary at the edge of the field, and without this a held tag would look like a fresh
# touch every time it happened.
MISSES_TO_LIFT = 2

# As long as `robotctl pad pair` looks by default. Someone is standing there with a pad in pairing
# mode; past this they have concluded it did not work.
PAIR_TIMEOUT_S = 15


def mac_in(records):
    """The first Bluetooth address in these NDEF records, as `AA:BB:CC:DD:EE:FF`, or None."""
    for record in records:
        found = MAC.search(record["valeur"])
        if found:
            octets = [found.group(i) for i in (1, 3, 4, 5, 6, 7)]
            return ":".join(octets).upper()
    return None


def robotctl(*args):
    """Run `robotctl <args> --json` and return its answer, or raise with what it said instead."""
    done = subprocess.run(
        ["robotctl", *args, "--json"], capture_output=True, text=True, check=False
    )
    if done.returncode != 0:
        raise RuntimeError((done.stderr or done.stdout).strip() or f"exit {done.returncode}")
    return json.loads(done.stdout)


def connected_pad(status):
    """The pad driving the robot now, from a `pad status` answer, or None."""
    return next((pad for pad in status["pads"] if pad["connected"]), None)


def on_touch(mac, log):
    """Everything that follows a tag carrying `mac`: check, pair, quack."""
    try:
        driving = connected_pad(robotctl("pad", "status"))
    except RuntimeError as exc:
        log(f"pad status failed, leaving it: {exc}")
        return
    if driving is not None:
        log(f"{driving['name']} {driving['mac']} is already connected — nothing to do")
        return

    log(f"pairing {mac} — the pad must be in pairing mode")
    try:
        result = robotctl("pad", "pair", mac, "--timeout", str(PAIR_TIMEOUT_S))
    except RuntimeError as exc:
        log(f"pad pair failed: {exc}")
        return
    if result["outcome"] != "paired":
        detail = f" ({result['detail']})" if result.get("detail") else ""
        log(f"not paired: {result['reason']}{detail}")
        return
    log(f"paired {result['pad']['name']} {result['pad']['mac']}")

    # `quack` has no `--json`: it prints a duck on success, and a refusal (a muted robot, no voice
    # bank) is a non-zero exit with the reason on stderr.
    quack = subprocess.run(["robotctl", "quack"], capture_output=True, text=True, check=False)
    if quack.returncode != 0:
        log(f"paired, but the quack was refused: {quack.stderr.strip()}")


def poll(chip, hz, log):
    """Watch the reader forever, calling `on_touch` once per arrival of a tag with an address."""
    from ntag663 import ndef, type2
    from ntag663.core import TagError
    from ntag663.iso14443a import select, wupa
    from ntag663.transport import TransportError

    period = 1.0 / hz
    present = None  # UID of the tag on the reader, already handled
    misses = 0

    while True:
        start = time.monotonic()
        try:
            chip.reset_field()
            wupa(chip)
            uid, _sak = select(chip)
        except TagError:
            uid = None
        except TransportError as exc:
            # A desynchronised link recovers with a soft reset, which `begin` starts with.
            log(f"reader link lost ({exc}), resetting")
            chip.begin()
            uid = None

        if uid is None:
            misses += 1
            if misses >= MISSES_TO_LIFT:
                present = None
        else:
            misses = 0
            if uid != present:
                present = uid
                try:
                    records = ndef.parse(type2.read_user_memory(chip))
                except (TagError, ndef.NdefError) as exc:
                    # Not marked handled: a tag read at the edge of the field gets another go on
                    # the next poll rather than needing to be lifted.
                    present = None
                    log(f"tag {uid.hex(':')} unreadable: {exc}")
                    records = None
                if records is not None:
                    mac = mac_in(records)
                    if mac is None:
                        log(f"tag {uid.hex(':')} carries no Bluetooth address: {records}")
                    else:
                        # The field is off while pairing blocks: no reason to power the antenna
                        # for the fifteen seconds nobody is reading it.
                        chip.field_off()
                        on_touch(mac, log)

        rest = period - (time.monotonic() - start)
        if rest > 0:
            time.sleep(rest)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyACM0", help="the reader's serial port")
    parser.add_argument("--hz", type=float, default=5.0, help="polls per second (default 5)")
    args = parser.parse_args()

    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass

    def log(message):
        print(f"[{time.strftime('%H:%M:%S')}] {message}")

    try:
        from ntag663.core import CLRC663
        from ntag663.transport import Transport
    except ImportError:
        print("ntag663 is not installed — pip install ~/winnie/lib_winnie", file=sys.stderr)
        return 1

    transport = Transport(port=args.port)
    try:
        transport.open()
    except OSError as exc:
        print(f"no reader on {args.port}: {exc}", file=sys.stderr)
        return 1
    chip = CLRC663(transport)
    try:
        chip.begin()
        log(f"reader on {args.port}, polling at {args.hz:g} Hz — touch a tag")
        poll(chip, args.hz, log)
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            chip.field_off()
        except BaseException:
            pass
        transport.close()


if __name__ == "__main__":
    sys.exit(main())
