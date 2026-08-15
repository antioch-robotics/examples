"""
Unitree Go2 locomotion scenario.

The Go2 walks on flat ground driven by Isaac Sim's pretrained flat-terrain
policy (isaacsim.robot.policy.examples). The run checks that the robot
actually walked: forward progress, height kept, uprightness, and bounded
lateral drift. A chase camera keeps the robot framed in the viewport so the
automatic telemetry capture shows the walk.

    antioch scenario run --scenario unitree_walk
"""

from __future__ import annotations

import math

import antioch

logger = antioch.Logger("unitree")

PHYSICS_DT = 1.0 / 200.0
RENDER_DT = 1.0 / 50.0


@antioch.scenario(
    tags=["unitree"],
    sim=antioch.BootProfile(physics_dt=PHYSICS_DT, render_dt=RENDER_DT, physics_engine="newton"),
    cases=[
        antioch.case(id="forward", tags=["smoke"]),
        antioch.case({"vx": 0.6, "wz": 0.5}, id="turn"),
    ],
)
def unitree_walk(
    run: antioch.ScenarioRun,
    vx: float = 1.0,
    wz: float = 0.0,
    duration_s: float = 8.0,
    settle_s: float = 1.5,
    ramp_s: float = 1.0,
) -> None:
    """Walk a Unitree Go2 with the flat-terrain policy and verify the gait."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    import numpy as np
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.robot.policy.examples.robots import Go2FlatTerrainPolicy
    from isaacsim.storage.native import get_assets_root_path
    from pxr import UsdLux, UsdPhysics, UsdShade

    world = antioch.world()
    stage = antioch.stage()

    # Match the policy's training configuration (NVIDIA's Go2 example):
    # torch backend on GPU, grid environment, friction-1.0 ground material.
    SimulationManager.set_backend("torch")
    SimulationManager.set_physics_sim_device("cuda")

    stage_utils.add_reference_to_stage(
        usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
        path="/World/ground",
    )
    # The grid env's light is local to the origin; a dome keeps the robot
    # evenly exposed as it walks away.
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(700.0)
    material = UsdShade.Material.Define(stage, "/World/ground/Looks/PhysicsMaterial")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(1.0)
    physics_material.CreateDynamicFrictionAttr().Set(1.0)
    physics_material.CreateRestitutionAttr().Set(0.0)
    ground_geom = stage.GetPrimAtPath("/World/ground/GroundPlane/CollisionPlane")
    if ground_geom.IsValid():
        UsdShade.MaterialBindingAPI.Apply(ground_geom).Bind(material)
    else:
        logger.warning("ground collision plane not found; friction material unbound")

    go2 = Go2FlatTerrainPolicy(prim_path="/World/Go2", position=[0.0, 0.0, 0.50])
    world.reset()

    import torch

    device = torch.device(str(go2.robot._device))
    command = torch.zeros(3, device=device)

    # initialize() must run inside the first physics step (canonical Go2
    # example): earlier calls leave joint state uncommitted and the robot
    # lands wrong and flips. The example drives the policy from
    # POST_PHYSICS_STEP so observations are current, not one step stale.
    from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents

    physics_ready = {"value": False}

    def on_physics_step(step_size: float, context: object = None) -> None:
        if physics_ready["value"]:
            go2.forward(step_size, command)
        else:
            physics_ready["value"] = True
            go2.initialize()
            go2.post_reset()

    policy_callback = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

    def base_pose() -> tuple[np.ndarray, float, float, float]:
        """World position, base height, tilt (deg), and yaw (rad) of the base link."""
        pos, quat = go2.robot.get_world_poses()
        p = pos.numpy().reshape(-1)[:3]
        w, x, y, z = quat.numpy().reshape(-1)[:4]  # Isaac Sim quaternions are WXYZ
        up_z = 1.0 - 2.0 * (x * x + y * y)
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return p, float(p[2]), tilt_deg, yaw

    def chase_camera(target: np.ndarray) -> None:
        eye = [float(target[0]) - 1.8, float(target[1]) - 1.8, 1.2]
        set_camera_view(eye=eye, target=[float(target[0]), float(target[1]), 0.3])

    def viewport_frame() -> "np.ndarray | None":
        pixels = antioch.capture_viewport()
        if pixels is None:
            return None
        return np.asarray(pixels)[..., :3].astype(np.uint8)

    def record_video_frame() -> None:
        frame = viewport_frame()
        if frame is not None:
            logger.image("camera/chase", frame, max_width=640)

    def snapshot(name: str) -> None:
        frame = viewport_frame()
        if frame is None:
            logger.warning(f"viewport capture returned no frame for {name}")
            return
        logger.image(f"snapshots/{name}", frame)
        try:
            from PIL import Image

            path = f"/tmp/{name}.png"
            Image.fromarray(frame).save(path)
            run.add_artifact(path, name=f"{name}.png", content_type="image/png")
        except Exception as exc:  # PNG artifact is a bonus; the RRD copy is kept
            logger.warning(f"could not save {name}.png: {exc}")

    # Settle: balance in place under zero command before judging anything.
    start_pos, _, _, _ = base_pose()
    chase_camera(start_pos)
    for i in range(int(settle_s / RENDER_DT)):
        world.step(render=True)
        if i % 5 == 0:  # 10 fps video of the whole run
            record_video_frame()
    settle_pos, settle_height, settle_tilt, settle_yaw = base_pose()
    run.check(
        "settled upright",
        settle_height > 0.20 and settle_tilt < 15.0,
        detail=f"height {settle_height:.3f} m > 0.20 m, tilt {settle_tilt:.1f}° < 15.0°",
    )
    snapshot("settled")

    # Walk: ramp the command in over ramp_s so the policy is not hit with a
    # velocity step, then hold (vx, wz).
    min_height = math.inf
    max_tilt = 0.0
    path_m = 0.0
    total_yaw = 0.0
    prev_p = settle_pos
    prev_yaw = settle_yaw
    render_steps = int(duration_s / RENDER_DT)
    for i in range(render_steps):
        ramp = min(1.0, (i * RENDER_DT) / ramp_s) if ramp_s > 0 else 1.0
        command[0] = vx * ramp
        command[2] = wz * ramp
        world.step(render=True)
        p, height, tilt, yaw = base_pose()
        min_height = min(min_height, height)
        max_tilt = max(max_tilt, tilt)
        path_m += float(np.linalg.norm((p - prev_p)[:2]))
        dyaw = yaw - prev_yaw
        total_yaw += math.atan2(math.sin(dyaw), math.cos(dyaw))  # unwrap
        prev_yaw = yaw
        if i % 5 == 0:
            record_video_frame()
            speed = float(np.linalg.norm((p - prev_p)[:2])) / RENDER_DT
            logger.scalar("base/x", float(p[0]))
            logger.scalar("base/y", float(p[1]))
            logger.scalar("base/height", height)
            logger.scalar("base/tilt_deg", tilt)
            logger.scalar("base/speed_mps", speed)
            logger.scalar("base/yaw_deg", math.degrees(total_yaw))
        prev_p = p
        chase_camera(p)
        if i == render_steps // 2:
            snapshot("mid_walk")

    end_pos, end_height, end_tilt, _ = base_pose()
    snapshot("end")
    SimulationManager.deregister_callback(policy_callback)

    # The policy tracks body-frame velocity with no heading feedback, so a
    # small yaw drift integrates into world-frame lateral offset; judge
    # distance covered along the path, and heading only when commanded.
    forward_m = float(end_pos[0] - settle_pos[0])
    effective_s = duration_s - 0.5 * ramp_s
    expected = abs(vx) * effective_s
    drift = abs(float(end_pos[1] - settle_pos[1]))
    expected_yaw = wz * effective_s

    run.add_results(
        {
            "command": {"vx": vx, "wz": wz, "duration_s": duration_s, "ramp_s": ramp_s},
            "forward_m": round(forward_m, 3),
            "path_m": round(path_m, 3),
            "expected_m": round(expected, 3),
            "min_height_m": round(min_height, 3),
            "max_tilt_deg": round(max_tilt, 1),
            "final_height_m": round(end_height, 3),
            "lateral_drift_m": round(drift, 3),
            "total_yaw_deg": round(math.degrees(total_yaw), 1),
            "expected_yaw_deg": round(math.degrees(expected_yaw), 1),
        }
    )

    run.check("covered distance", path_m >= 0.5 * expected, detail=f"path {path_m:.2f} m >= {0.5 * expected:.2f} m (commanded {expected:.2f} m)")
    run.check("kept height", min_height >= 0.15, detail=f"min height {min_height:.3f} m >= 0.150 m")
    run.check("stayed upright", max_tilt <= 30.0, detail=f"max tilt {max_tilt:.1f}° <= 30.0°")
    if wz == 0.0:
        run.check("walked straight ahead", forward_m >= 0.5 * expected, detail=f"forward {forward_m:.2f} m >= {0.5 * expected:.2f} m")
        run.check("bounded drift", drift <= 1.0, detail=f"lateral drift {drift:.2f} m <= 1.00 m")
    else:
        turned_enough = abs(total_yaw) >= 0.5 * abs(expected_yaw) and total_yaw * expected_yaw > 0
        run.check("turned as commanded", turned_enough, detail=f"yaw {math.degrees(total_yaw):.0f}° vs commanded {math.degrees(expected_yaw):.0f}° (gate 50%)")
