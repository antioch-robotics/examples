# Isaac Sim 6.0.1 examples

A small Antioch project showing the three ways to run Isaac Sim code on a
remote GPU machine: a plain script, recorded scenarios, and suites. All
simulation runs remotely — nothing here needs Isaac Sim installed locally.

## What's here

| File | What it is |
|---|---|
| `src/main.py` | A plain script (not a scenario): rains cubes onto a ground plane in a loop, mainly to exercise the livestream. |
| `src/scenarios.py` | Two recorded scenarios: `falling_cube`, a fast smoke check that a dropped cube settles, and `cube_bounce`, a 6-case parameter sweep (3 drop heights × 2 restitutions) measuring rebound. |
| `src/unitree.py` | `unitree_walk`: a Unitree Go2 walks on flat ground using Isaac Sim's pretrained flat-terrain policy, with a chase camera, logged telemetry, and checks on distance, height, uprightness, and drift. Two cases: `forward` and `turn`. |
| `src/so101_teleop.py` | `so101_live_teleop`: mirrors a physical SO-101 leader arm live in sim. Listens on a TCP port inside the sim container and applies streamed joint frames as position targets. |
| `teleop/leader_bridge.py` | The laptop half of teleop: reads the physical SO-101 leader arm with lerobot and streams its joints through the port tunnel into the scenario. |
| `antioch.yaml` | The project manifest: the `sim` service (image `antioch-engine/isaac-sim-6.0.1`), the teleop port tunnel, and two suites, `smoke` and `sweep`. |
| `pyproject.toml` | Python 3.12 project depending on `antioch-sim[isaac-sim]` and `lerobot[feetech]`, managed with uv. |

## Setup

```bash
uv sync
```

Then either activate the environment (`source .venv/bin/activate`) or prefix
the commands below with `uv run`. You'll need to be signed in to Antioch
(`antioch auth login`); machine allocation happens automatically on first run.

## Running things

**The plain script** — output and exit status are the whole story, nothing is
recorded. Streams a live viewport by default:

```bash
antioch run src/main.py                   # 30s run with livestream
antioch run --no-stream src/main.py       # headless
antioch run src/main.py -- --seconds 5    # quick iteration
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
```

Add `--queue` to any scenario or suite run to execute it unattended (headless,
survives closing the terminal).

**Live SO-101 teleop** — drive the sim arm with the physical leader arm, in
three terminals:

```bash
antioch services up                              # 1. stack + the teleop port tunnel
antioch scenario run --scenario so101_live_teleop   # 2. the cloud half (streams live)
uv run python teleop/leader_bridge.py            # 3. the laptop half (auto-detects the arm)
```

The scenario listens on TCP 56321 inside the sim container; the `ports` entry
in `antioch.yaml` tunnels it to `localhost:56321`, and the bridge streams the
leader's joints (degrees; gripper 0–100) into it at 30 Hz. Watch the sim arm
follow your hand on the machine livestream (`antioch machine status` prints
the URL). Bridge hotkeys: `r` resets and randomizes the cube, `q` ends the
cloud session, Ctrl-C exits the bridge but leaves the session waiting for a
reconnect until its `max_seconds` (default 600 s) elapses.

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
