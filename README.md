# examples

Example projects for [Antioch](https://antioch.com), the simulation platform
for physical AI. Each directory is a self-contained Antioch project — an
`antioch.yaml` plus ordinary Python — that runs on a remote GPU machine, so
nothing here needs a simulator installed locally.

| Example | What it shows |
|---|---|
| [`isaac-sim-6.0.1/`](isaac-sim-6.0.1/) | Isaac Sim 6.0.1: a livestreamed script, recorded scenarios (a falling-cube smoke check, a cube-bounce parameter sweep, and a Unitree Go2 walking on a pretrained policy), and the `smoke`/`sweep` suites. |
| `isaac-lab-3/` | Isaac Lab 3.0 (coming soon). |

## Getting started

Each example manages its own environment with [uv](https://docs.astral.sh/uv/):

```bash
cd isaac-sim-6.0.1
uv sync
uv run antioch auth login   # first time only
uv run antioch run src/main.py
```

See each example's README for what it contains and the full set of ways to
run it.
