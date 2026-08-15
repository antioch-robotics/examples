"""Stream the SO-101 leader arm's joints to the cloud teleop scenario, ~30 Hz.

The so101_live_teleop scenario listens on a TCP port inside the sim
container; `antioch services up` opens an authenticated tunnel for it on
localhost (see the `ports` entry in antioch.yaml). This bridge reads the
physical leader arm with lerobot and streams newline-delimited JSON frames
into that tunnel. Run order:

    1. uv run antioch services up
    2. uv run antioch scenario run --scenario so101_live_teleop
    3. uv run python teleop/leader_bridge.py            # this script

The serial port is auto-detected when exactly one /dev/tty.usbmodem* is
present; pass --port to disambiguate. Frames are the ROBOT convention
(degrees; gripper 0..100) declared in a header line; the sim side converts
and refuses a units mismatch. Hotkeys: [r] reset + randomize the scene,
[q] stop the cloud session and exit, Ctrl-C exit without stopping it.
"""

import argparse
import glob
import importlib
import importlib.util
import json
import socket
import sys
import time

# newest lerobot first: so_leader is the unified SO-100/101 driver in >=0.6,
# so101_leader is the standalone module in older releases
LEADER_DRIVERS = [
    ("so_leader", "lerobot.teleoperators.so_leader", "SOLeader"),
    ("so101_leader", "lerobot.teleoperators.so101_leader", "SO101Leader"),
]
KEYS = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
        "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]


def detect_driver():
    for name, mod_name, cls in LEADER_DRIVERS:
        if importlib.util.find_spec(mod_name) is not None:
            mod = importlib.import_module(mod_name)
            Leader = getattr(mod, cls)
            # the driver's own config_class is the complete config (the
            # module-level *Config export can be a partial base class)
            print(f"# leader driver: {name}", file=sys.stderr)
            return Leader, Leader.config_class, name
    raise SystemExit("no lerobot SO leader driver found — run `uv sync` in the project root")


def detect_serial_port() -> str:
    ports = sorted(glob.glob("/dev/tty.usbmodem*"))
    if len(ports) != 1:
        raise SystemExit(f"expected exactly one /dev/tty.usbmodem* port, found {ports or 'none'}; "
                         "pass --port explicitly")
    return ports[0]


def make_leader(Leader, Cfg, serial_port: str, arm_id: str, calibrate: bool):
    try:
        cfg = Cfg(port=serial_port, id=arm_id, use_degrees=True)
    except TypeError:
        try:
            cfg = Cfg(port=serial_port, use_degrees=True)
        except TypeError as exc:
            raise SystemExit(f"leader config schema mismatch ({exc}) — report, don't patch")
    if getattr(cfg, "use_degrees", None) is not True:
        raise SystemExit("leader config did not accept use_degrees=True — refusing")
    leader = Leader(cfg)
    leader.connect(calibrate=calibrate)
    if not leader.is_calibrated:
        leader.disconnect()
        raise SystemExit("leader arm is not calibrated — rerun with --calibrate for the "
                         "interactive lerobot calibration flow")
    return leader


def connect_tcp(host: str, port: int, header: str) -> socket.socket:
    """Connect to the scenario's tunnel, retrying until the scenario acks.

    The tunnel accepts connections locally even before the scenario listens
    on the far side, so a successful connect proves nothing — only the
    scenario's ack line in response to the header does.
    """
    waited = False
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=3.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.sendall(header.encode() + b"\n")
            sock.settimeout(5.0)
            ack = b""
            while b"\n" not in ack:
                chunk = sock.recv(256)
                if not chunk:
                    raise OSError("closed before ack")
                ack += chunk
            if not json.loads(ack.split(b"\n", 1)[0]).get("ack"):
                raise OSError(f"unexpected reply {ack!r}")
            sock.settimeout(None)
            print(f"# connected to {host}:{port} (scenario acked)", file=sys.stderr)
            return sock
        except (OSError, ValueError):
            if not waited:
                print(f"# waiting for {host}:{port} — is `antioch services up` running and "
                      "the so101_live_teleop scenario started?", file=sys.stderr)
                waited = True
            time.sleep(1.0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=None, help="serial port of the LEADER arm (auto-detected if omitted)")
    ap.add_argument("--id", default="my_leader_arm", help="lerobot calibration id")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--host", default="127.0.0.1", help="tunnel host")
    ap.add_argument("--tcp-port", type=int, default=56321,
                    help="tunnel port (must match antioch.yaml and the scenario's listen_port)")
    ap.add_argument("--calibrate", action="store_true",
                    help="allow the interactive lerobot calibration flow on connect")
    args = ap.parse_args()

    serial_port = args.port or detect_serial_port()
    Leader, Cfg, driver = detect_driver()
    leader = make_leader(Leader, Cfg, serial_port, args.id, args.calibrate)
    header = json.dumps({"header": True, "driver": driver,
                         "units": {"joints": "degrees", "gripper": "0_100"},
                         "keys": KEYS, "hz": args.hz})
    sock = connect_tcp(args.host, args.tcp_port, header)
    print(f"# streaming at {args.hz} Hz — hotkeys: [r] reset+randomize scene, "
          "[q] stop session and exit, Ctrl-C exit (session keeps waiting)", file=sys.stderr)

    # raw single-key reads from the terminal without blocking the stream
    import select
    import termios
    import tty
    tty_ok = sys.stdin.isatty()
    if tty_ok:
        old_attrs = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def send(payload: dict) -> None:
        nonlocal sock
        line = json.dumps(payload).encode() + b"\n"
        try:
            sock.sendall(line)
        except OSError:
            print("# connection lost, reconnecting…", file=sys.stderr)
            sock.close()
            sock = connect_tcp(args.host, args.tcp_port, header)
            sock.sendall(line)

    period = 1.0 / args.hz
    n = 0
    t0 = time.monotonic()
    try:
        while True:
            if tty_ok and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == "r":
                    send({"cmd": "reset"})
                    print("# reset sent", file=sys.stderr)
                elif key == "q":
                    send({"cmd": "stop"})
                    print("# stop sent — ending session", file=sys.stderr)
                    break
            act = leader.get_action()
            missing = [k for k in KEYS if k not in act]
            if missing:
                raise SystemExit(f"leader action missing keys {missing} — got {sorted(act)}")
            row = {k: round(float(act[k]), 3) for k in KEYS}
            send({"t": round(time.monotonic() - t0, 3), **row})
            n += 1
            if n % 300 == 0:
                print(f"# {n} frames sent", file=sys.stderr)
            time.sleep(max(0.0, t0 + n * period - time.monotonic()))
    except KeyboardInterrupt:
        print("# stopped (session keeps waiting until its max_seconds)", file=sys.stderr)
    finally:
        if tty_ok:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        sock.close()
        leader.disconnect()


if __name__ == "__main__":
    main()
