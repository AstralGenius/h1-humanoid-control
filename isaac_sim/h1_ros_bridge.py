"""H1 Humanoid Control — Isaac Sim ↔ ROS 2 Bridge (OmniGraph-based).

Stage 2.C — Full bridge:
  Subscribes /h1/cmd_vel (geometry_msgs/Twist) → policy command
  Publishes  /h1/odom    (nav_msgs/Odometry)   ← robot state
  Publishes  /clock      (rosgraph_msgs/Clock) ← sim time

See docs/interface.md for the contract.
"""

# SimulationApp must be created first
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable the ROS 2 bridge extension before importing from it
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import numpy as np
import carb
import omni.graph.core as og
import usdrt.Sdf

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

from bridge_config import (
    PHYSICS_DT, RENDERING_DT, ROBOT_SPAWN_HEIGHT,
    H1_PRIM_PATH, GRAPH_PATH, NODE_NAMESPACE,
    CMD_VEL_TOPIC, ODOM_TOPIC, ODOM_FRAME, BASE_FRAME,
    MAX_LIN_X, MAX_ANG_Z,
)

settings = carb.settings.get_settings()
settings.set("/app/runLoops/main/rateLimitEnabled", True)
settings.set("/app/runLoops/main/rateLimitFrequency", 60)  # cap renderer at 60 fps
settings.set("/physics/autoPopupSimulationOutputWindow", False)

# ---------------------------------------------------------------------------
# World & robot setup
# ---------------------------------------------------------------------------
world = World(
    stage_units_in_meters=1.0,
    physics_dt=PHYSICS_DT,
    rendering_dt=RENDERING_DT,
)

assets_root = get_assets_root_path()
if assets_root is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    raise SystemExit(1)

env_prim = define_prim("/World/Ground", "Xform")
env_prim.GetReferences().AddReference(
    assets_root + "/Isaac/Environments/Grid/default_environment.usd"
)

h1 = H1FlatTerrainPolicy(
    prim_path=H1_PRIM_PATH,
    name="H1",
    usd_path=assets_root + "/Isaac/Robots/Unitree/H1/h1.usd",
    position=np.array([0.0, 0.0, ROBOT_SPAWN_HEIGHT]),
)

# ---------------------------------------------------------------------------
# OmniGraph: clock + cmd_vel subscribe + odom publish
# ---------------------------------------------------------------------------
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime",       "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock",      "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubscribeTwist",    "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("ComputeOdometry",   "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdometry",   "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ],
        keys.CONNECT: [
            # Clock pipeline
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

            # Subscribe pipeline
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),

            # Odometry pipeline
            ("OnPlaybackTick.outputs:tick",        "ComputeOdometry.inputs:execIn"),
            ("ComputeOdometry.outputs:execOut",    "PublishOdometry.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),

            # Wire the computed state into the publish node
            ("ComputeOdometry.outputs:position",         "PublishOdometry.inputs:position"),
            ("ComputeOdometry.outputs:orientation",      "PublishOdometry.inputs:orientation"),
            ("ComputeOdometry.outputs:linearVelocity",   "PublishOdometry.inputs:linearVelocity"),
            ("ComputeOdometry.outputs:angularVelocity",  "PublishOdometry.inputs:angularVelocity"),
        ],
        keys.SET_VALUES: [
            # Subscribe config
            ("SubscribeTwist.inputs:topicName",    CMD_VEL_TOPIC),
            ("SubscribeTwist.inputs:nodeNamespace", NODE_NAMESPACE),

            # Odometry source
            ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(H1_PRIM_PATH)]),

            # Publish config
            ("PublishOdometry.inputs:topicName",     ODOM_TOPIC),
            ("PublishOdometry.inputs:nodeNamespace", NODE_NAMESPACE),
            ("PublishOdometry.inputs:odomFrameId",   ODOM_FRAME),
            ("PublishOdometry.inputs:chassisFrameId", BASE_FRAME),
        ],
    },
)

# Cache subscribe attribute handles
twist_node_path = f"{GRAPH_PATH}/SubscribeTwist"
linear_attr = og.Controller.attribute(f"{twist_node_path}.outputs:linearVelocity")
angular_attr = og.Controller.attribute(f"{twist_node_path}.outputs:angularVelocity")

# ---------------------------------------------------------------------------
# Physics callback
# ---------------------------------------------------------------------------
first_step = True


def on_physics_step(step_size: float) -> None:
    global first_step
    if first_step:
        h1.initialize()
        first_step = False
        return

    lin = linear_attr.get()
    ang = angular_attr.get()

    lin_x = float(np.clip(lin[0], 0.0, MAX_LIN_X))
    ang_z = float(np.clip(ang[2], -MAX_ANG_Z, MAX_ANG_Z))

    cmd = np.array([lin_x, 0.0, ang_z], dtype=np.float32)
    h1.forward(step_size, cmd)


world.reset()
world.add_physics_callback("physics_step", callback_fn=on_physics_step)

print(f"[bridge 2.C] Full bridge active:")
print(f"  Subscribed:  {NODE_NAMESPACE}/{CMD_VEL_TOPIC}")
print(f"  Published:   {NODE_NAMESPACE}/{ODOM_TOPIC}, /clock")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
try:
    while simulation_app.is_running():
        world.step(render=True)
finally:
    simulation_app.close()
