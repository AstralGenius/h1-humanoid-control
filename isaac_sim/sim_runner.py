"""Isaac Sim world, robot, and physics loop for the H1 bridge.

Pure simulation logic — does not import rclpy.
Takes a state_sink callable (push pose/velocity out) and a command_source
callable (pull velocity command in), both supplied by the caller.
"""

import numpy as np
import carb

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

from bridge_config import PHYSICS_DT, RENDERING_DT, ROBOT_SPAWN_HEIGHT


class H1SimRunner:
    """Owns the Isaac Sim world, the H1 robot, and the physics callback."""

    def __init__(self, env_url: str, command_source, state_sink):
        """
        Parameters
        ----------
        env_url : str
            Environment USD path, relative to the assets root.
        command_source : callable () -> np.ndarray
            Returns the current [lin_x, lin_y, ang_z] command.
        state_sink : callable (pos, quat, lin_vel, ang_vel) -> None
            Receives the robot's current state each physics step.
        """
        self._command_source = command_source
        self._state_sink = state_sink
        self._first_step = True
        self._reset_needed = False

        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=PHYSICS_DT,
            rendering_dt=RENDERING_DT,
        )

        assets_root = get_assets_root_path()
        if assets_root is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            raise RuntimeError("Assets root not found")

        # Environment
        ground = define_prim("/World/Ground", "Xform")
        ground.GetReferences().AddReference(assets_root + env_url)

        # Robot
        self._h1 = H1FlatTerrainPolicy(
            prim_path="/World/H1",
            name="H1",
            usd_path=assets_root + "/Isaac/Robots/Unitree/H1/h1.usd",
            position=np.array([0.0, 0.0, ROBOT_SPAWN_HEIGHT]),
        )

        self._world.reset()
        self._world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)

    def _on_physics_step(self, step_size: float) -> None:
        if self._first_step:
            self._h1.initialize()
            self._first_step = False
            return
        if self._reset_needed:
            self._world.reset(True)
            self._reset_needed = False
            self._first_step = True
            return

        # Drive policy with the latest command
        self._h1.forward(step_size, self._command_source())

        # Push state out
        art = self._h1.robot
        pos, quat = art.get_world_pose()
        self._state_sink(pos, quat, art.get_linear_velocity(), art.get_angular_velocity())

    def step(self) -> None:
        """Advance one render step. Call this in the main loop."""
        self._world.step(render=True)
        if self._world.is_stopped():
            self._reset_needed = True
