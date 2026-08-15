"""
Example scenarios for Isaac Lab: one fast check, and one parameter sweep.

    antioch scenario run --scenario cartpole_balance    one run
    antioch suite run smoke                  the fast check
    antioch suite run sweep                  every case on one machine
    antioch suite run sweep --machines 4     opt into multi-machine fan-out

The sweep shows how one scenario declaration expands into independent cases.
Queue staging adds the submitted project source to the immutable sim image;
add a Dockerfile only when the project needs custom packages or another image
layer. Development watch rules are not part of queued runs.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import antioch

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.scene import InteractiveScene
    from isaaclab.sim import SimulationContext

logger = antioch.Logger("cartpole")


def build_cartpole() -> tuple[SimulationContext, InteractiveScene, Articulation]:
    """
    Bring up one cartpole on a ground plane, ready to step.

    Isaac Lab imports live inside the body, not at module scope: that is the
    rule that lets this file be discovered and type-checked on a laptop with
    no simulator installed.

    :return: The simulation context, its scene, and the cartpole articulation.
    """

    import isaaclab.sim as sim_utils
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg
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
    return simulation, scene, scene["cartpole"]


def drive(simulation: SimulationContext, scene: InteractiveScene, robot: Articulation, step: int, amplitude: float, period: float) -> None:
    """
    Advance one physics step under a sinusoidal cart effort.

    :param simulation: The running simulation context.
    :param scene: The interactive scene holding the cartpole.
    :param robot: The cartpole articulation.
    :param step: Index of this step, which sets the phase.
    :param amplitude: Peak effort applied to the cart joint.
    :param period: Steps per full effort cycle.
    """

    import torch  # pyright: ignore[reportMissingImports] — the Isaac Lab engine ships torch

    effort = amplitude * math.sin(2.0 * math.pi * step / period)
    robot.set_joint_effort_target_index(target=torch.full_like(robot.data.joint_pos[:, [0]], effort), joint_ids=[0])
    scene.write_data_to_sim()
    simulation.step()
    scene.update(simulation.get_physics_dt())


@antioch.scenario(tags=["smoke"])
def cartpole_balance(run: antioch.ScenarioRun, steps: int = antioch.param(300, ge=1, description="Physics steps to simulate")) -> None:
    """
    Push an Isaac Lab cartpole and verify that its state stays finite.
    """

    simulation, scene, robot = build_cartpole()
    for step in range(steps):
        drive(simulation, scene, robot, step, amplitude=2.0, period=180.0)
        logger.scalar("pole_angle", float(robot.data.joint_pos[0, 1]))

    rail_x = float(robot.data.root_pos_w[0, 0])
    final_angle = float(robot.data.joint_pos[0, 1])
    run.add_result("final_pole_angle", round(final_angle, 4))
    # Both criteria are recorded even when the first one fails, so a run that
    # went non-finite still reports where the cart ended up
    run.check("the pole angle stayed finite", math.isfinite(final_angle), detail=f"final angle {final_angle}")
    run.check("the cart stayed within a metre of the origin", abs(rail_x) < 1.0, detail=f"cart at x={rail_x:.3f} m")


@antioch.scenario(tags=["sweep"], cases=[antioch.case(grid={"amplitude": [0.5, 2.0, 6.0], "period": [30.0, 120.0]}, id="a{amplitude}-p{period}")])
def cartpole_drive(
    run: antioch.ScenarioRun,
    amplitude: float = antioch.param(2.0, ge=0.1, le=20.0, description="Peak effort applied to the cart joint"),
    period: float = antioch.param(60.0, ge=2.0, description="Steps per full effort cycle"),
    steps: int = antioch.param(300, ge=1, description="Physics steps to simulate"),
) -> None:
    """
    Drive the cart at varying amplitude and frequency, and bound the swing.

    Six children from one declaration: three efforts against two frequencies.
    """

    simulation, scene, robot = build_cartpole()
    swing = 0.0
    for step in range(steps):
        drive(simulation, scene, robot, step, amplitude=amplitude, period=period)
        angle = float(robot.data.joint_pos[0, 1])
        logger.scalar("pole_angle", angle)
        swing = max(swing, abs(angle))

    run.add_result("peak_pole_angle", round(swing, 4))
    run.check("the pole angle stayed finite", math.isfinite(swing), detail=f"peak swing {swing} under amplitude {amplitude}")
