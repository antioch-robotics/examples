# Antioch Examples

Example projects for [Antioch](https://antioch.com), the simulation platform
for physical AI. Each directory is a self-contained Antioch project — an
`antioch.yaml` plus ordinary Python — that runs on a remote GPU machine, so
nothing here needs a simulator installed locally.

| Example | What it shows |
|---|---|
| [`isaac-sim-6.0.1/`](isaac-sim-6.0.1/) | Isaac Sim 6.0.1, one folder per example: `sandbox/` (a naked GUI session), `cubes/` (a livestreamed script, a falling-cube smoke check, and a cube-bounce parameter sweep), `unitree/` (a Go2 walking on a pretrained policy), and `so101-teleop/` (a physical SO-101 leader arm mirrored live in sim). Suites: `smoke`, `sweep`, `cubes`, `unitree`. |
| [`isaac-lab-3/`](isaac-lab-3/) | Isaac Lab 3.0, one folder per example: `sandbox/` (a naked GUI session), `cartpole/` (a livestreamed script, a smoke check, and a parameter sweep), and `unitree/` (a Go2 joint-space choreography). Suites: `smoke`, `sweep`, `cartpole`, `unitree`. |

## Getting started

Each example manages its own environment with [uv](https://docs.astral.sh/uv/):

```bash
cd isaac-sim-6.0.1        # or isaac-lab-3
uv sync
uv run antioch auth login   # first time only
uv run antioch run cubes/demo.py
```

Interactive runs stream the simulator GUI by default; `antioch machine
status` prints the stream URL. See each example's README for what it
contains and the full set of ways to run it.
