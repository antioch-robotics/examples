"""
Example scenarios for Isaac Sim: one fast check, and one parameter sweep.

    antioch scenario run --scenario falling_cube        one run
    antioch suite run smoke                  the fast check
    antioch suite run sweep                  every case on one machine
    antioch suite run sweep --machines 4     opt into multi-machine fan-out

The sweep shows how one scenario declaration expands into independent cases.
Queue staging adds the submitted project source to the immutable sim image;
add a Dockerfile only when the project needs custom packages or another image
layer. Development watch rules are not part of queued runs.
"""

from __future__ import annotations

import antioch

logger = antioch.Logger("cube")


@antioch.scenario(tags=["smoke"])
def falling_cube(
    run: antioch.ScenarioRun,
    drop_height: float = antioch.param(2.0, ge=0.5, le=10.0, description="Initial cube height in meters"),
    steps: int = antioch.param(180, ge=1, description="Physics steps to simulate"),
) -> None:
    """
    Drop a dynamic cube and verify that it settles on the ground.
    """

    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid

    world = antioch.world()
    world.scene.add_ground_plane(restitution=0.0)
    cube = world.scene.add(
        DynamicCuboid(prim_path="/World/cube", name="cube", position=np.array([0.0, 0.0, drop_height]), size=0.5, color=np.array([0.2, 0.4, 0.9]))
    )
    world.reset()
    for _ in range(steps):
        world.step(render=False)
        logger.scalar("height", float(cube.get_world_pose()[0][2]))

    final_z = float(cube.get_world_pose()[0][2])
    run.add_result("final_z", round(final_z, 4))
    run.check("the cube came to rest on the ground", final_z < 0.4, detail=f"cube centre rested at {final_z:.3f} m")


@antioch.scenario(tags=["sweep"], cases=[antioch.case(grid={"drop_height": [0.5, 2.0, 6.0], "restitution": [0.0, 0.7]}, id="h{drop_height}-e{restitution}")])
def cube_bounce(
    run: antioch.ScenarioRun,
    drop_height: float = antioch.param(2.0, ge=0.1, le=10.0, description="Initial cube height in meters"),
    restitution: float = antioch.param(0.0, ge=0.0, le=1.0, description="How bouncy the ground is"),
    steps: int = antioch.param(240, ge=1, description="Physics steps to simulate"),
) -> None:
    """
    Drop a cube onto ground of varying bounciness and measure the rebound.

    Six children from one declaration: three heights against two materials.
    """

    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid

    world = antioch.world()
    world.scene.add_ground_plane(restitution=restitution)
    cube = world.scene.add(
        DynamicCuboid(prim_path="/World/cube", name="cube", position=np.array([0.0, 0.0, drop_height]), size=0.3, color=np.array([0.9, 0.4, 0.2]))
    )
    world.reset()

    # The rebound is the highest point AFTER the cube first reaches the floor,
    # so the drop itself is never mistaken for a bounce
    landed = False
    rebound = 0.0
    for _ in range(steps):
        world.step(render=False)
        height = float(cube.get_world_pose()[0][2])
        logger.scalar("height", height)
        landed = landed or height < 0.2
        if landed:
            rebound = max(rebound, height)

    run.add_result("rebound_height", round(rebound, 4))
    # Both criteria are recorded even when the first one fails, so a run that
    # never reached the floor still reports what its rebound measured
    run.check("the cube reached the floor", landed, detail=f"within {steps} steps from {drop_height:.2f} m")
    run.check("the cube rebounded no higher than it was dropped", rebound < drop_height, detail=f"rebounded to {rebound:.3f} m from {drop_height:.2f} m")
