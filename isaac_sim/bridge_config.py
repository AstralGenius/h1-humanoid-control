"""H1 bridge configuration. Mirrors docs/interface.md."""

# Policy limits
MAX_LIN_X = 0.75            # m/s, forward only
MAX_ANG_Z = 0.75            # rad/s

# Topic names
CMD_VEL_TOPIC = "cmd_vel"   # OmniGraph nodes prepend the namespace below
ODOM_TOPIC = "odom"
NODE_NAMESPACE = "/h1"

# Frames
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# Sim parameters
PHYSICS_DT = 1.0 / 200.0
RENDERING_DT = 8.0 / 200.0
ROBOT_SPAWN_HEIGHT = 1.05

# Stage paths
H1_PRIM_PATH = "/World/H1"
GRAPH_PATH = "/World/H1_ROS_Bridge"
