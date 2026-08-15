"""
Unitree Go2 choreography demo for Isaac Lab 3.

The Go2 is spawned straight from its USD asset — no InteractiveScene, no
policy — and driven through a looping joint-space routine: settle into
stance, squats, hip sway, and a trot in place. Every act is a target pattern
around the stance pose held by implicit PD actuators, so the demo exercises
the raw articulation path: spawn, actuator config, position targets, step.
An orbiting camera keeps the livestream interesting while the robot stays
near the origin.

    antioch run src/unitree.py                          livestream two loops
    antioch run src/unitree.py -- --seconds 90          keep it running longer
    antioch scenario run --scenario unitree_go2_routine one recorded run
    antioch suite run unitree                           the unitree suite
"""

from __future__ import annotations

import argparse
import math
from typing import TYPE_CHECKING

import antioch

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationContext

logger = antioch.Logger("unitree")

PHYSICS_DT = 1.0 / 200.0
RENDER_EVERY = 4  # render at 50 fps out of the 200 Hz physics loop

# One pass through the routine: stand, two squats, three sways, then trot.
SETTLE_S = 2.0
SQUAT_S = 6.0
SQUAT_PERIOD_S = 3.0
SWAY_S = 6.0
SWAY_PERIOD_S = 2.0
TROT_S = 8.0
TROT_HZ = 1.8
TROT_RAMP_S = 1.0
ROUTINE_S = SETTLE_S + SQUAT_S + SWAY_S + TROT_S

# Joint-space amplitudes, all relative to the stance pose. Thigh and calf
# move together so the foot stays under the hip as the leg folds.
SQUAT_THIGH_RAD = 0.35
SQUAT_CALF_RAD = 0.60
SWAY_HIP_RAD = 0.08
# Quick, small steps: large slow lifts rock the body on the loaded diagonal
# and it is still recovering seconds after the trot ends.
STEP_THIGH_RAD = 0.15
STEP_CALF_RAD = 0.25

CAMERA_ORBIT_S = 40.0
CAMERA_RADIUS_M = 2.0
CAMERA_HEIGHT_M = 1.0


def build_go2() -> tuple[SimulationContext, Articulation]:
    """
    Bring up one Unitree Go2 on a ground plane, ready to step.

    Isaac Lab imports live inside the body, not at module scope: that is the
    rule that lets this file be discovered and type-checked on a laptop with
    no simulator installed.

    :return: The simulation context and the Go2 articulation.
    """

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

    # The engine image ships the default Newton physics backend only; the
    # optional ovphysx wheel is not installed, so no physics= override here.
    simulation = SimulationContext(SimulationCfg(dt=PHYSICS_DT, device="cuda:0"))

    ground = sim_utils.GroundPlaneCfg(physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0))
    ground.func("/World/GroundPlane", ground)
    light = sim_utils.DomeLightCfg(intensity=2200.0)
    light.func("/World/Light", light)

    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Go2",
            spawn=sim_utils.UsdFileCfg(
                usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.42),
                # The Go2 stance pose; without it the USD default (straight
                # legs) makes the robot land badly before folding into place.
                joint_pos={
                    ".*L_hip_joint": 0.1,
                    ".*R_hip_joint": -0.1,
                    "F[L,R]_thigh_joint": 0.8,
                    "R[L,R]_thigh_joint": 1.0,
                    ".*_calf_joint": -1.5,
                },
            ),
            # Stiffer than the Go2's RL-policy gains (25/0.5): a trained
            # policy compensates for gravity, but pure position holds sag
            # under it — the loaded rear legs droop and the body pitches
            # back ~20° at stance.
            actuators={"all": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=80.0, damping=3.0)},
        )
    )
    simulation.reset()
    return simulation, robot


def perform(simulation: SimulationContext, robot: Articulation, seconds: float) -> dict[str, float]:
    """
    Run the looping choreography for the given duration and gather stats.

    :param simulation: The running simulation context.
    :param robot: The Go2 articulation.
    :param seconds: How long to run; the routine loops every ``ROUTINE_S``.
    :return: Height and tilt statistics over the whole run.
    """

    dt = simulation.get_physics_dt()
    default = robot.data.default_joint_pos.clone()
    names = robot.joint_names
    legs = {leg: {part: names.index(f"{leg}_{part}_joint") for part in ("hip", "thigh", "calf")} for leg in ("FL", "FR", "RL", "RR")}

    def base_state() -> tuple[float, float, float, float]:
        """Base x, y, height (m) and tilt from vertical (deg)."""
        px, py, height = (float(v) for v in robot.data.root_pos_w[0, :3])
        w, x, y, z = (float(v) for v in robot.data.root_quat_w[0])
        up_z = 1.0 - 2.0 * (x * x + y * y)
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
        return px, py, height, tilt_deg

    min_height = math.inf
    max_tilt = 0.0
    steps = int(round(seconds / dt))
    for step in range(steps):
        t = step * dt
        phase = t % ROUTINE_S
        targets = default.clone()

        if phase < SETTLE_S:
            pass  # hold stance
        elif phase < SETTLE_S + SQUAT_S:
            s = 0.5 * (1.0 - math.cos(2.0 * math.pi * (phase - SETTLE_S) / SQUAT_PERIOD_S))
            for joints in legs.values():
                targets[:, joints["thigh"]] += SQUAT_THIGH_RAD * s
                targets[:, joints["calf"]] -= SQUAT_CALF_RAD * s
        elif phase < SETTLE_S + SQUAT_S + SWAY_S:
            roll = SWAY_HIP_RAD * math.sin(2.0 * math.pi * (phase - SETTLE_S - SQUAT_S) / SWAY_PERIOD_S)
            for joints in legs.values():
                targets[:, joints["hip"]] += roll
        else:
            trot_t = phase - SETTLE_S - SQUAT_S - SWAY_S
            ramp = min(1.0, trot_t / TROT_RAMP_S)
            osc = math.sin(2.0 * math.pi * TROT_HZ * trot_t)
            for leg, joints in legs.items():
                lift = ramp * max(0.0, osc if leg in ("FL", "RR") else -osc)
                targets[:, joints["thigh"]] += STEP_THIGH_RAD * lift
                targets[:, joints["calf"]] -= STEP_CALF_RAD * lift

        robot.set_joint_position_target(targets)
        robot.write_data_to_sim()
        render = step % RENDER_EVERY == 0
        simulation.step(render=render)
        robot.update(dt)

        px, py, height, tilt = base_state()
        min_height = min(min_height, height)
        max_tilt = max(max_tilt, tilt)
        if render:
            # Orbit around wherever the robot actually is, so drift during
            # the trot never walks it out of frame.
            angle = 0.25 * math.pi + 2.0 * math.pi * t / CAMERA_ORBIT_S
            eye = [px + CAMERA_RADIUS_M * math.cos(angle), py + CAMERA_RADIUS_M * math.sin(angle), CAMERA_HEIGHT_M]
            simulation.set_camera_view(eye, [px, py, 0.3])
        if render and (step // RENDER_EVERY) % 4 == 0:  # ~12 Hz telemetry
            logger.scalar("base/height_m", height)
            logger.scalar("base/tilt_deg", tilt)
            logger.scalar("base/drift_m", math.hypot(px, py))

    px, py, final_height, final_tilt = base_state()
    return {
        "min_height_m": min_height,
        "max_tilt_deg": max_tilt,
        "final_height_m": final_height,
        "final_tilt_deg": final_tilt,
        "drift_m": math.hypot(px, py),
        "steps": float(steps),
    }


@antioch.scenario(tags=["unitree"])
def unitree_go2_routine(
    run: antioch.ScenarioRun,
    seconds: float = antioch.param(24.0, ge=4.0, le=300.0, description="How long to run the looping routine"),
) -> None:
    """
    Run the Go2 through its choreography and verify that it kept its feet.
    """

    simulation, robot = build_go2()
    stats = perform(simulation, robot, seconds)

    run.add_results({key: round(value, 3) for key, value in stats.items()})
    run.check("state stayed finite", all(math.isfinite(value) for value in stats.values()), detail=f"stats {stats}")
    run.check("kept height", stats["min_height_m"] >= 0.10, detail=f"min height {stats['min_height_m']:.3f} m >= 0.100 m")
    run.check("stayed upright", stats["max_tilt_deg"] <= 30.0, detail=f"max tilt {stats['max_tilt_deg']:.1f}° <= 30.0°")
    run.check("ended upright", stats["final_tilt_deg"] <= 20.0, detail=f"final tilt {stats['final_tilt_deg']:.1f}° <= 20.0°")


def main() -> None:
    """
    Livestream the Go2 choreography until the deadline.

    Streaming is on by default. Pass ``--no-stream`` to ``antioch run`` for a
    headless run; this same file supports both launch modes.
    """

    parser = argparse.ArgumentParser(description="Run the Unitree Go2 choreography demo")
    parser.add_argument("--seconds", type=float, default=2.0 * ROUTINE_S, help="How long to simulate before exiting")
    arguments = parser.parse_args()

    antioch.boot()

    simulation, robot = build_go2()
    stats = perform(simulation, robot, arguments.seconds)
    print(
        f"ran the Go2 routine for {int(stats['steps'])} steps over {arguments.seconds:g}s: "
        f"min height {stats['min_height_m']:.3f} m, max tilt {stats['max_tilt_deg']:.1f}°"
    )


if __name__ == "__main__":
    main()
