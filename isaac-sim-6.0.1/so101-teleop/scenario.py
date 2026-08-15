"""
so101_live_teleop — mirror a physical SO-101 leader arm in sim, live.

Transport: this scenario listens on a TCP port inside the sim container
(services run with host networking), and `antioch.yaml` declares that port
as an authenticated tunnel. The local bridge (so101-teleop/leader_bridge.py)
connects to localhost:<port> and streams newline-delimited JSON frames in
the robot's native units (degrees; gripper 0..100). The scenario converts
to sim radians against the articulation's actual DOF limits and applies
them as position targets every physics tick.

Run order (three terminals):

    1. uv run antioch services up                       # stack + port tunnel
    2. uv run antioch scenario run --scenario so101_live_teleop
    3. uv run python so101-teleop/leader_bridge.py            # auto-detects the arm

Watch the machine livestream (`antioch machine status` prints the URL).
The session ends when the bridge sends a stop command (press q) or after
max_seconds. Bridge hotkeys: [r] reset + randomize the cube, [q] stop.
"""

from __future__ import annotations

import antioch

logger = antioch.Logger("teleop")

# One frame from the bridge carries these keys, robot convention:
# five arm joints in degrees, gripper in percent open (0 closed, 100 open).
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT = "gripper"
KEYS = [f"{j}.pos" for j in ARM_JOINTS + [GRIPPER_JOINT]]
EXPECTED_UNITS = {"joints": "degrees", "gripper": "0_100"}

# lerobot joint name -> candidate DOF names in the USD asset; so101_antioch
# keeps the original SO-ARM100 URDF names (Rotation, Pitch, ... Jaw)
SIM_DOF_ALIASES = {
    "shoulder_pan": ("shoulder_pan", "Rotation"),
    "shoulder_lift": ("shoulder_lift", "Pitch"),
    "elbow_flex": ("elbow_flex", "Elbow"),
    "wrist_flex": ("wrist_flex", "Wrist_Pitch"),
    "wrist_roll": ("wrist_roll", "Wrist_Roll"),
    "gripper": ("gripper", "Jaw"),
}


@antioch.scenario(
    tags=["so101", "teleop", "isaacsim"],
    sim=antioch.BootProfile(physics_dt=1.0 / 60.0, render_dt=1.0 / 60.0),
)
def so101_live_teleop(
    run: antioch.ScenarioRun,
    robot_asset: str = antioch.param("so101_antioch", description="SO-101 robot USD on the shelf"),
    robot_version: str = antioch.param("1.3.2", description="Robot asset version (1.3.2 is the validated configuration)"),
    listen_port: int = antioch.param(56321, ge=1024, le=65535, description="TCP port the bridge connects to; must match the antioch.yaml ports entry"),
    max_seconds: float = antioch.param(600.0, ge=10.0, description="Session length ceiling"),
    cube_size: float = antioch.param(0.04, ge=0.02, le=0.08, description="Cube edge (m)"),
    dr_seed: int = antioch.param(7, description="Seed for reset randomization"),
    render_every: int = antioch.param(2, ge=1, description="Render every N physics ticks (2 = 30 Hz at dt 1/60)"),
) -> None:
    """Follow the streamed leader joints until the bridge stops or the clock runs out."""

    import json
    import random
    import socket
    import threading
    import time

    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.experimental.prims import Articulation
    from pxr import Usd, UsdLux, UsdPhysics

    # --- shared state between the socket thread and the sim loop ------------
    lock = threading.Lock()
    shared = {"latest": None, "header": None, "frames": 0, "connections": 0}
    commands: list[dict] = []
    stop_event = threading.Event()

    def serve() -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", listen_port))
        srv.listen(1)
        srv.settimeout(1.0)
        try:
            while not stop_event.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                with lock:
                    shared["connections"] += 1
                logger.info(f"bridge connected from {addr[0]}:{addr[1]}")
                conn.settimeout(1.0)
                buf = b""
                try:
                    while not stop_event.is_set():
                        try:
                            chunk = conn.recv(65536)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            if line.strip():
                                handle_line(line, conn)
                finally:
                    conn.close()
                    logger.info("bridge disconnected; listening for reconnect")
        finally:
            srv.close()

    def handle_line(line: bytes, conn: socket.socket) -> None:
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            return  # torn line mid-write
        if row.get("header"):
            if row.get("units") != EXPECTED_UNITS:
                logger.error(f"bridge units {row.get('units')!r} != {EXPECTED_UNITS}; ignoring its frames")
                return
            with lock:
                shared["header"] = row
            # the tunnel accepts connections locally even before this server
            # listens, so the bridge waits for this ack to know it's through
            try:
                conn.sendall(b'{"ack": true}\n')
            except OSError:
                pass
            logger.info(f"bridge header ok: driver={row.get('driver')} hz={row.get('hz')}")
            return
        if "cmd" in row:
            with lock:
                commands.append(row)
            return
        if all(k in row for k in KEYS):
            with lock:
                if shared["header"] is not None:
                    shared["latest"] = row
                    shared["frames"] += 1

    # --- scene ---------------------------------------------------------------
    world = antioch.world()
    stage = antioch.stage()
    world.scene.add_ground_plane(z_position=0.0, restitution=0.0)
    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1500.0)

    antioch.load_asset(robot_asset, prim_path="/World/so101", version=robot_version)

    rng = random.Random(dr_seed)
    cube_home = np.array([0.25, -0.08, cube_size / 2 + 0.005])
    cube = world.scene.add(
        DynamicCuboid(prim_path="/World/cube", name="cube", position=cube_home, size=cube_size, mass=0.04, color=np.array([0.15, 0.60, 0.25]))
    )

    world.reset()

    # the asset's articulation root may sit below the reference prim
    root_path = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/so101")):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root_path = str(prim.GetPath())
            break
    if root_path is None:
        raise RuntimeError(f"no ArticulationRootAPI found under /World/so101 in asset {robot_asset}@{robot_version}")
    robot = Articulation(root_path)

    dof_names = list(robot.dof_names)
    dof_index: dict[str, int] = {}
    for joint, candidates in SIM_DOF_ALIASES.items():
        matches = [c for c in candidates if c in dof_names]
        if len(matches) != 1:
            raise RuntimeError(f"asset DOFs {dof_names} have no unique match for {joint!r} among {candidates}")
        dof_index[joint] = dof_names.index(matches[0])
    lower, upper = (a.numpy()[0] for a in robot.get_dof_limits())
    g = dof_index[GRIPPER_JOINT]
    logger.info(f"articulation at {root_path}: dofs={dof_names} gripper limits=[{lower[g]:.3f}, {upper[g]:.3f}] rad")

    # aim the default viewport (= the livestream) at the workspace
    try:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=[0.75, -0.55, 0.55], target=[0.22, 0.0, 0.08])
    except Exception as exc:  # noqa: BLE001
        logger.warning(f"viewport aim failed: {exc}")

    def to_targets(row: dict, out: np.ndarray) -> None:
        # arm joints: leader degrees -> radians, clipped into the sim's limits
        for name in ARM_JOINTS:
            i = dof_index[name]
            out[i] = float(np.clip(np.radians(float(row[f"{name}.pos"])), lower[i], upper[i]))
        # gripper: percent open mapped onto the actual DOF range (0 = closed)
        pct = float(np.clip(row[f"{GRIPPER_JOINT}.pos"], 0.0, 100.0))
        out[g] = lower[g] + (pct / 100.0) * (upper[g] - lower[g])

    def scene_reset() -> None:
        x = rng.uniform(0.20, 0.28)
        y = rng.uniform(-0.15, 0.05)
        cube.set_world_pose(np.array([x, y, cube_size / 2 + 0.01]))
        try:
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
        except Exception:  # noqa: BLE001
            pass
        logger.info(f"scene reset: cube -> ({x:.3f}, {y:.3f})")

    server = threading.Thread(target=serve, name="teleop-server", daemon=True)
    server.start()
    logger.info(f"listening on port {listen_port}; start the bridge on your laptop now")

    targets = robot.get_dof_positions().numpy()[0].copy()
    stop_requested = False
    ticks = 0
    last_report = time.monotonic()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < max_seconds and not stop_requested:
            with lock:
                pending, commands[:] = commands[:], []
                row = shared["latest"]
            for cmd in pending:
                if cmd.get("cmd") == "reset":
                    scene_reset()
                elif cmd.get("cmd") == "stop":
                    stop_requested = True
                else:
                    logger.warning(f"ignoring unknown command {cmd!r}")
            if row is not None:
                to_targets(row, targets)
            robot.set_dof_position_targets(targets.reshape(1, -1))
            world.step(render=(ticks % render_every == 0))
            ticks += 1
            if ticks % 30 == 0 and row is not None:
                for name in ARM_JOINTS + [GRIPPER_JOINT]:
                    logger.scalar(f"target/{name}", float(targets[dof_index[name]]))
            if time.monotonic() - last_report > 10.0:
                with lock:
                    frames, conns = shared["frames"], shared["connections"]
                logger.info(f"t={time.monotonic() - t0:.0f}s frames_in={frames} ticks={ticks} connections={conns}")
                last_report = time.monotonic()
    finally:
        stop_event.set()
        server.join(timeout=3.0)

    with lock:
        header, frames, conns = shared["header"], shared["frames"], shared["connections"]
    run.add_results(
        {
            "session_seconds": round(time.monotonic() - t0, 1),
            "leader_frames": frames,
            "ticks": ticks,
            "connections": conns,
            "stopped_by_bridge": stop_requested,
        }
    )
    run.check("the bridge connected and declared its units", header is not None, detail=f"{conns} connection(s); header {'seen' if header else 'never seen'}")
    run.check("leader frames were received and mirrored", frames > 0, detail=f"{frames} frames over {ticks} ticks")
