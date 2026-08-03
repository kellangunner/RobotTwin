"""RobotTwin — run a text script of commands against the firmware over serial.

Reads a plain-text file whose lines are the same host->robot protocol commands
you would otherwise type into `pio device monitor` (PING, ENABLE, SETHOME,
MOVEJ, MOVEL, GRIP, PAYLOAD, TELEM, HOME, STOP, DISABLE, STATE), sends them one
at a time, and waits for each to fully complete before sending the next -- an
accepted move blocks until the firmware reports `EV MOVE_DONE`, a HOME until
`EV HOMED`, a GRIP until `EV GRIP_DONE`. Any `ERR` or `EV FAULT` stops the run
(unless --continue-on-error).

The one transformation this runner adds: **a MOVEJ that moves more than one
joint is executed one joint at a time.** The bench rig shares a single motor
rail across all three drivers, so three coils energizing together sags the
supply; sequencing the joints keeps only one motor moving at any instant. The
held joints are re-commanded to their current angle each sub-move (so they draw
only holding current, not motion current). MOVEL cannot be split host-side (it
resolves to a pose via the firmware's inverse kinematics), so it is sent whole
with a warning; write multi-joint moves as MOVEJ to get the sequencing.

Script format:
    - one command per line, exactly as typed at the console
    - blank lines and lines starting with '#' are ignored
    - 'SLEEP <seconds>' pauses the host between commands (runner directive)

Usage:
    python python/run_script.py --port COM5 script.txt
    python python/run_script.py --dry-run --start 0,90,-90 script.txt

--dry-run needs no hardware: it prints the exact wire lines that would be sent
(including the per-joint decomposition) so a script can be previewed off-bench.

Requires pyserial for live runs:  pip install pyserial
"""

import argparse
import sys
import time

JOINT_NAMES = ("base", "shoulder", "elbow")

# Verbs whose OK is only an acceptance ack; the command is not finished until a
# terminal event arrives on its own line some time later.
MOTION_VERBS = {"MOVEJ", "MOVEL"}   # -> EV MOVE_DONE
HOME_VERBS = {"HOME", "SETHOME"}    # -> EV HOMED
GRIP_VERBS = {"GRIP"}               # -> EV GRIP_DONE (the jaws slew, they
                                    #    do not jump to the commanded opening)

# Two joint targets are "the same" within this many degrees (well under one
# microstep at every joint's resolution, so it never masks a real move).
ANGLE_EPS_DEG = 1e-3


# ---------------------------------------------------------------------------
# Serial line I/O: buffer bytes and hand back whole newline-terminated lines,
# tolerating reads that land mid-line (telemetry and events interleave freely
# with command replies on the same wire).
# ---------------------------------------------------------------------------

class SerialLink:
    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def write_line(self, text):
        self.ser.write((text + "\n").encode("ascii"))
        self.ser.flush()

    def read_line(self, deadline):
        """Next full line, or None once `deadline` (monotonic seconds) passes."""
        while True:
            nl = self.buf.find(b"\n")
            if nl >= 0:
                line = self.buf[:nl]
                self.buf = self.buf[nl + 1:]
                return line.decode("ascii", "replace").strip()
            if time.monotonic() > deadline:
                return None
            chunk = self.ser.read(64)
            if chunk:
                self.buf += chunk


class Reply:
    def __init__(self, ok, detail=""):
        self.ok = ok
        self.detail = detail


# ---------------------------------------------------------------------------
# Script parsing
# ---------------------------------------------------------------------------

class Line:
    def __init__(self, number, verb, args, raw):
        self.number = number
        self.verb = verb
        self.args = args      # tokens after the verb
        self.raw = raw


def parse_script(path):
    lines = []
    with open(path, "r", encoding="utf-8") as f:
        for i, raw in enumerate(f, start=1):
            text = raw.strip()
            if not text or text.startswith("#"):
                continue
            tok = text.split()
            lines.append(Line(i, tok[0].upper(), tok[1:], text))
    return lines


def parse_order(spec):
    """'base,shoulder,elbow' (or indices) -> a permutation of [0,1,2]."""
    out = []
    for part in spec.split(","):
        key = part.strip().lower()
        if key in JOINT_NAMES:
            idx = JOINT_NAMES.index(key)
        elif key in ("0", "1", "2"):
            idx = int(key)
        else:
            raise ValueError(f"bad joint in --order: {part!r}")
        if idx in out:
            raise ValueError(f"joint repeated in --order: {part!r}")
        out.append(idx)
    if len(out) != 3:
        raise ValueError("--order must list all three joints")
    return out


# ---------------------------------------------------------------------------
# Multi-joint MOVEJ -> sequential single-joint moves
# ---------------------------------------------------------------------------

def decompose_movej(target, pose, order, split):
    """Return the list of full 3-joint MOVEJ targets to send, in order.

    One entry if the move touches at most one joint (or splitting is off);
    otherwise one entry per changed joint, each holding the others at their
    current angle so only that joint moves. Empty if nothing changes.
    """
    changed = [j for j in range(3) if abs(target[j] - pose[j]) > ANGLE_EPS_DEG]
    if not changed:
        return []
    if not split or len(changed) == 1:
        return [list(target)]

    seq = []
    cur = list(pose)
    for j in order:
        if j in changed:
            cur[j] = target[j]
            seq.append(list(cur))
    return seq


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------

class Executor:
    def __init__(self, link, order, split, timeout, dry):
        self.link = link
        self.order = order
        self.split = split
        self.timeout = timeout
        self.dry = dry
        self.pose = [0.0, 0.0, 0.0]   # tracked joint angles, degrees

    def send(self, wire):
        """Send one already-formatted wire line; wait for it to complete."""
        verb = wire.split()[0].upper()
        print(f"  -> {wire}")
        if self.dry:
            return Reply(True)
        self.link.write_line(wire)
        return self._await(verb)

    def _await(self, verb):
        deadline = time.monotonic() + self.timeout
        acked = False   # for motion/home verbs: OK seen, still waiting for EV
        while True:
            line = self.link.read_line(deadline)
            if line is None:
                return Reply(False, f"timeout waiting for {verb} to complete")
            if not line:
                continue
            print(f"  <- {line}")
            head = line.split()

            if head[0] == "ST":
                self._update_pose_from_st(head)
                if verb == "STATE":
                    return Reply(True)
                continue
            if head[0] == "ERR":
                return Reply(False, line)
            if head[0] == "PONG":
                if verb == "PING":
                    return Reply(True)
                continue
            if head[0] == "OK":
                if verb in MOTION_VERBS or verb in HOME_VERBS or verb in GRIP_VERBS:
                    acked = True   # keep reading for the terminal event
                    continue
                return Reply(True)
            if head[0] == "EV":
                name = head[1] if len(head) > 1 else ""
                if name == "FAULT":
                    return Reply(False, line)
                if name == "MOVE_DONE" and verb in MOTION_VERBS:
                    return Reply(True)
                if name == "HOMED" and verb in HOME_VERBS:
                    return Reply(True)
                if name == "GRIP_DONE" and verb in GRIP_VERBS:
                    return Reply(True)
                # BOOT / HOMING / other progress events: keep waiting.
                continue
            # Unknown line: ignore and keep waiting.
            _ = acked

    def _update_pose_from_st(self, tokens):
        # ST <b> <s> <e> <mode> homed=.. en=.. payload=.. grip=..
        try:
            self.pose = [float(tokens[1]), float(tokens[2]), float(tokens[3])]
        except (IndexError, ValueError):
            pass

    def query_state(self):
        """Refresh the tracked pose from the firmware (no-op in dry-run)."""
        if self.dry:
            return Reply(True)
        return self.send("STATE")

    def run_line(self, ln):
        if ln.verb == "SLEEP":
            secs = float(ln.args[0]) if ln.args else 0.0
            print(f"  .. sleep {secs}s")
            if not self.dry:
                time.sleep(secs)
            return Reply(True)

        if ln.verb == "MOVEJ":
            return self._run_movej(ln)

        r = self.send(ln.raw)
        if not r.ok:
            return r
        # Keep the tracked pose honest after datum / homing changes.
        if ln.verb == "SETHOME":
            self.pose = [float(a) for a in ln.args]
        elif ln.verb == "HOME":
            self.query_state()
        return r

    def _run_movej(self, ln):
        if len(ln.args) != 3:
            return Reply(False, "MOVEJ expects 3 joint angles")
        try:
            target = [float(a) for a in ln.args]
        except ValueError:
            return Reply(False, f"bad MOVEJ arguments: {ln.args}")

        seq = decompose_movej(target, self.pose, self.order, self.split)
        if not seq:
            print("  .. already at target, nothing to move")
            return Reply(True)

        if len(seq) > 1:
            moved = [JOINT_NAMES[j] for j in self.order
                     if abs(target[j] - self.pose[j]) > ANGLE_EPS_DEG]
            print(f"  splitting multi-joint MOVEJ into {len(seq)} moves "
                  f"({', '.join(moved)})")

        for i, step in enumerate(seq, start=1):
            if len(seq) > 1:
                print(f"  step {i}/{len(seq)}")
            r = self.send("MOVEJ %.4f %.4f %.4f" % (step[0], step[1], step[2]))
            if not r.ok:
                return r
            self.pose = list(step)
        return Reply(True)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("script", help="path to the command script (.txt)")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="seconds to wait for a single command to complete")
    ap.add_argument("--order", default="base,shoulder,elbow",
                    help="joint order for split moves (default proximal->distal)")
    ap.add_argument("--no-split", action="store_true",
                    help="send multi-joint MOVEJ as one coordinated move")
    ap.add_argument("--continue-on-error", action="store_true",
                    help="keep running after an ERR / FAULT instead of stopping")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the wire lines without opening a serial port")
    ap.add_argument("--start", default="0,0,0",
                    help="assumed start pose for --dry-run, 'b,s,e' in degrees")
    args = ap.parse_args(argv)

    try:
        order = parse_order(args.order)
    except ValueError as e:
        ap.error(str(e))

    try:
        script = parse_script(args.script)
    except OSError as e:
        print(f"cannot read script: {e}", file=sys.stderr)
        return 2
    if not script:
        print("script has no commands", file=sys.stderr)
        return 2

    link = None
    ser = None
    if not args.dry_run:
        if not args.port:
            ap.error("--port is required unless --dry-run is given")
        try:
            import serial  # pyserial
        except ImportError:
            print("pyserial not installed. Run: pip install pyserial",
                  file=sys.stderr)
            return 2
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
        except serial.SerialException as e:
            print(f"cannot open {args.port}: {e}", file=sys.stderr)
            return 2
        link = SerialLink(ser)

    ex = Executor(link, order, not args.no_split, args.timeout, args.dry_run)

    try:
        if args.dry_run:
            ex.pose = [float(a) for a in args.start.split(",")]
            print(f"[dry run] assumed start pose: {ex.pose} deg")
        else:
            print(f"opened {args.port} @ {args.baud}")
            ex.query_state()   # seed the pose from the live datum

        failures = 0
        for ln in script:
            print(f"line {ln.number}: {ln.raw}")
            r = ex.run_line(ln)
            if not r.ok:
                failures += 1
                print(f"  ** FAILED: {r.detail}", file=sys.stderr)
                if not args.continue_on_error:
                    print("stopping (use --continue-on-error to keep going)",
                          file=sys.stderr)
                    return 1
        return 1 if failures else 0
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    sys.exit(main())
