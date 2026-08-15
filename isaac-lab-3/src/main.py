"""
Drive a cartpole under a sine effort, so a livestream is visibly live.

    antioch run src/main.py                   run with the default livestream
    antioch run --no-stream src/main.py       run headlessly
    antioch run src/main.py -- --seconds 5    quick enough to iterate on
    antioch run src/main.py -- --seconds 600  long enough to walk away from

Every knob is an argument because the point of this file is to be dispatched
many ways, often at once: concurrent runs of different lengths are what
exercise a project's machines, the console's stream picker, and the
interactive quota.
"""

from __future__ import annotations

import argparse
import math
import time

import antioch


def main() -> None:
    """
    Step a cartpole scene with rendering on until the deadline.

    Streaming is on by default.  Pass ``--no-stream`` for a headless run; this
    same file supports both launch modes.
    """

    parser = argparse.ArgumentParser(description="Drive a cartpole under a sine effort")
    parser.add_argument("--seconds", type=float, default=30.0, help="How long to simulate before exiting")
    arguments = parser.parse_args()

    antioch.boot()

    import isaaclab.sim as sim_utils
    import torch  # pyright: ignore[reportMissingImports] — the Isaac Lab engine ships torch
    from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaaclab.utils import configclass
    from isaaclab_assets.robots.cartpole import CARTPOLE_CFG

    @configclass
    class CartpoleSceneCfg(InteractiveSceneCfg):
        """
        One cartpole on a ground plane under a dome light.
        """

        ground = AssetBaseCfg(prim_path="/World/GroundPlane", spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2500.0))
        cartpole: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Cartpole")

    simulation = SimulationContext(SimulationCfg(dt=1.0 / 60.0))
    scene = InteractiveScene(CartpoleSceneCfg(num_envs=1, env_spacing=2.0))
    simulation.reset()
    robot: Articulation = scene["cartpole"]

    # Rendering every step is the whole point: a stream of a static frame says
    # nothing about whether streaming works.
    deadline = time.monotonic() + arguments.seconds
    step = 0
    while time.monotonic() < deadline:
        effort = 2.0 * math.sin(step / 30.0)
        robot.set_joint_effort_target_index(target=torch.full_like(robot.data.joint_pos[:, [0]], effort), joint_ids=[0])
        scene.write_data_to_sim()
        simulation.step()
        scene.update(simulation.get_physics_dt())
        step += 1
    print(f"drove the cartpole for {step} steps over {arguments.seconds:g}s")


if __name__ == "__main__":
    main()
