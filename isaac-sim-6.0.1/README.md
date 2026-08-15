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
| `antioch.yaml` | The project manifest: the `sim` service (image `antioch-engine/isaac-sim-6.0.1`) and two suites, `smoke` and `sweep`. |
| `pyproject.toml` | Python 3.12 project depending on `antioch-sim[isaac-sim]`, managed with uv. |

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

**Reading results back:**

```bash
antioch scenario show SCENARIO_RUN_ID --json    # verdict, checks, results
antioch scenario logs SCENARIO_RUN_ID
antioch scenario download SCENARIO_RUN_ID       # telemetry (.rrd) and artifacts
```

When you're done iterating, `antioch machine release` frees the GPU machine.
