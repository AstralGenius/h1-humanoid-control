#!/bin/bash
# Launch the Isaac Sim ↔ ROS 2 OmniGraph bridge.

set -eo pipefail

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source Isaac Sim's internal ROS 2 setup (LD_LIBRARY_PATH, RMW, ROS_DISTRO)
set +u
source "$ISAAC_SIM_ROOT/setup_ros_env.sh"
set -u

exec "$ISAAC_SIM_ROOT/python.sh" "$PROJECT_ROOT/isaac_sim/h1_ros_bridge.py" "$@"
