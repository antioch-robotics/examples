# Isaac Lab 3.0 examples

A small Antioch project showing Isaac Lab 3.0 code running on a remote GPU
machine: plain scripts, recorded scenarios, and suites. All simulation runs
remotely — nothing here needs Isaac Lab installed locally.

## What's here

One folder per example, plus the project files:

| Path | What it contains |
|---|---|
| `sandbox/` | `demo.py` boots a naked Isaac Lab session and idles the event loop, so the streamed GUI is fully yours: build a scene, drop in assets, press Play. Nothing is scripted and nothing is recorded. |
| `cartpole/` | Cartpole basics. `demo.py` is a plain script (not a scenario) that drives a cartpole under a sine effort, mainly to exercise the livestream. `scenarios.py` holds `cartpole_balance`, a fast smoke check that the state stays finite, and `cartpole_drive`, a 6-case parameter sweep (3 efforts × 2 frequencies) bounding the swing. |
| `unitree/` | `routine.py` holds `unitree_go2_routine`: a Go2 spawned straight from its USD asset runs a looping joint-space choreography — stance, squats, hip sway, and a trot in place — with checks on height and uprightness. The same file doubles as a plain script for a pure livestream run. |
| `antioch.yaml` | The project manifest: the `sim` service (image `antioch-engine/isaac-lab-3.0`) and the suites below. |
| `pyproject.toml` | Python 3.12 project depending on `antioch-sim[isaac-lab]`, managed with uv. |

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
antioch run cartpole/demo.py                   # 30s run with livestream
antioch run --no-stream cartpole/demo.py       # headless
antioch run unitree/routine.py -- --seconds 90 # the Go2 choreography, longer
antioch run sandbox/demo.py                    # a naked GUI session (15 min)
antioch run --timeout 86400 sandbox/demo.py    # ... that lasts all day
```

**Scenarios** — each run is recorded with pass/fail checks, results,
telemetry, and logs you can read back later:

```bash
antioch scenario collect                       # preview what's defined
antioch scenario run --scenario cartpole_balance
antioch scenario run --scenario unitree_go2_routine
```

**Suites** — named selections from `antioch.yaml`:

```bash
antioch suite run smoke                   # the fast cartpole check
antioch suite run sweep                   # all 6 cartpole_drive cases
antioch suite run sweep --machines 4      # fan the sweep out across machines
antioch suite run cartpole                # everything in cartpole/
antioch suite run unitree                 # the Go2 choreography
```

Add `--queue` to any scenario or suite run to execute it unattended (headless,
survives closing the terminal).

**Reading results back:**

```bash
antioch scenario show SCENARIO_RUN_ID --json    # verdict, checks, results
antioch scenario logs SCENARIO_RUN_ID
antioch scenario download SCENARIO_RUN_ID       # telemetry (.rrd) and artifacts
```

When you're done iterating, `antioch machine release` frees the GPU machine.
