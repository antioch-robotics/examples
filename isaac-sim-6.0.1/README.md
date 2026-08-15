# Isaac Sim 6.0.1 examples

A small Antioch project showing Isaac Sim 6.0.1 code running on a remote GPU
machine: plain scripts, recorded scenarios, and suites. All simulation runs
remotely — nothing here needs Isaac Sim installed locally.

## What's here

One folder per example, plus the project files:

| Path | What it contains |
|---|---|
| `sandbox/` | `demo.py` boots a naked Isaac Sim session and idles the event loop, so the streamed GUI is fully yours: build a scene, drop in assets, press Play. Nothing is scripted and nothing is recorded. |
| `cubes/` | Cube physics. `demo.py` is a plain script (not a scenario) that rains cubes onto a ground plane, mainly to exercise the livestream. `scenarios.py` holds `falling_cube`, a fast smoke check that a dropped cube settles, and `cube_bounce`, a 6-case parameter sweep (3 drop heights × 2 restitutions) measuring rebound. |
| `unitree/` | `walk.py` holds `unitree_walk`: a Unitree Go2 walks on flat ground using Isaac Sim's pretrained flat-terrain policy, with a chase camera, logged telemetry, and checks on distance, height, uprightness, and drift. Two cases: `forward` and `turn`. |
| `so101-teleop/` | Live teleop of a physical SO-101 arm. `scenario.py` holds `so101_live_teleop`, which listens on a TCP port inside the sim container and mirrors streamed joint frames as position targets; `leader_bridge.py` is the laptop half, reading the leader arm with lerobot and streaming its joints through the port tunnel. |
| `antioch.yaml` | The project manifest: the `sim` service (image `antioch-engine/isaac-sim-6.0.1`), the teleop port tunnel, and the suites below. |
| `pyproject.toml` | Python 3.12 project depending on `antioch-sim[isaac-sim]` and `lerobot[feetech]`, managed with uv. |

## Setup

```bash
uv sync
```

Then either activate the environment (`source .venv/bin/activate`) or prefix
the commands below with `uv run`. You'll need to be signed in to Antioch
(`antioch auth login`); machine allocation happens automatically on first run.

## Running things

Interactive runs stream the simulator GUI by default; `antioch machine status`
prints the stream URL.

**The plain scripts** — output and exit status are the whole story, nothing is
recorded:

```bash
antioch run cubes/demo.py                   # 30s run with livestream
antioch run --no-stream cubes/demo.py       # headless
antioch run cubes/demo.py -- --seconds 5    # quick iteration
antioch run sandbox/demo.py                 # a naked GUI session (15 min)
antioch run --timeout 86400 sandbox/demo.py # ... that lasts all day
```

**Scenarios** — each run is recorded with pass/fail checks, results,
telemetry, and logs you can read back later:

```bash
antioch scenario collect                       # preview what's defined
antioch scenario run --scenario falling_cube
antioch scenario run --scenario unitree_walk
```

**Suites** — named selections from `antioch.yaml`:

```bash
antioch suite run smoke                   # fast checks (falling_cube + unitree_walk's forward case)
antioch suite run sweep                   # all 6 cube_bounce cases
antioch suite run sweep --machines 4      # fan the sweep out across machines
antioch suite run cubes                   # everything in cubes/
antioch suite run unitree                 # both unitree_walk cases
```

The teleop example has no suite: it needs a human moving the physical leader
arm, so it is dispatched directly (below).

Add `--queue` to any scenario or suite run to execute it unattended (headless,
survives closing the terminal).

**Live SO-101 teleop** — drive the sim arm with the physical leader arm, in
three terminals:

```bash
antioch services up                                 # 1. stack + the teleop port tunnel
antioch scenario run --scenario so101_live_teleop   # 2. the cloud half (streams live)
python so101-teleop/leader_bridge.py                # 3. the laptop half (auto-detects the arm)
```

The scenario listens on TCP 56321 inside the sim container; the `ports` entry
in `antioch.yaml` tunnels it to `localhost:56321`, and the bridge streams the
leader's joints (degrees; gripper 0–100) into it at 30 Hz. Watch the sim arm
follow your hand on the machine livestream. Bridge hotkeys: `r` resets and
randomizes the cube, `q` ends the cloud session, Ctrl-C exits the bridge but
leaves the session waiting for a reconnect until its `max_seconds` (default
600 s, raise with `--set max_seconds=1800`) elapses. If the bridge can't
connect after the machine changed, rerun `antioch services up` to re-open the
tunnel.

Arm auto-detection constraints — for it to work smoothly, **plug in only the
leader arm**:

- The bridge picks the serial port by globbing `/dev/tty.usbmodem*` and only
  proceeds when exactly one device matches. A second USB-serial device — the
  follower arm, or any dev board that enumerates the same way — makes it
  refuse rather than guess; pass `--port /dev/tty.usbmodemXXXX` to
  disambiguate (the name embeds the board's USB serial, so it's stable across
  replugs).
- The port alone doesn't say "leader": if you point it at the follower by
  mistake, the motors' stored calibration won't match the leader's
  calibration file and the bridge exits with a "not calibrated" error —
  correct outcome, misleading message.
- The calibration id defaults to `my_leader_arm` (the file under
  `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/`); use
  `--id` if your arm was calibrated under a different name, or `--calibrate`
  to run lerobot's interactive calibration flow.

**Reading results back:**

```bash
antioch scenario show SCENARIO_RUN_ID --json    # verdict, checks, results
antioch scenario logs SCENARIO_RUN_ID
antioch scenario download SCENARIO_RUN_ID       # telemetry (.rrd) and artifacts
```

When you're done iterating, `antioch machine release` frees the GPU machine.
