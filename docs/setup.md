# Setup Guide

End-to-end install and run for the H1 humanoid control project on a fresh Ubuntu 24.04 machine with an NVIDIA RTX GPU.

Expected time: 45–60 minutes (excluding Isaac Sim download, which is 12–13 GB and depends on your network).

---

## 1. System Requirements

- **OS:** Ubuntu 24.04 LTS (this guide assumes 24.04 — earlier versions need different ROS distros)
- **GPU:** NVIDIA RTX with at least 8 GB VRAM (developed on RTX 5090, 24 GB)
- **Driver:** NVIDIA driver 580 or later with open kernel modules
- **Disk:** ~30 GB free (Isaac Sim assets + ROS install + workspace)
- **Network:** for first-run asset downloads (~5–10 GB from NVIDIA Nucleus)

Verify the driver:

```bash
nvidia-smi
```

You should see your GPU listed with driver version 580+ and CUDA 12.6+.

---

## 2. Install ROS 2 Jazzy

ROS 2 Jazzy is the supported distro for Ubuntu 24.04.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

Verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

Should print the Jazzy version.

---

## 3. Install Isaac Sim 5.1

Download from NVIDIA:

1. Open [https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)
2. Follow the Linux Workstation Installation link to get the 5.1 zip (~13 GB)

Install:

```bash
mkdir -p ~/isaacsim
cd ~/isaacsim
unzip ~/Downloads/isaac-sim-*.zip
./post_install.sh
```

Verify:

```bash
~/isaacsim/python.sh --version
```

Should print `Python 3.11.13`.

---

## 4. Clone the Project

```bash
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/AstralGenius/h1-humanoid-control.git
cd h1-humanoid-control
```

---

## 5. Build the ROS 2 Workspace

```bash
cd ~/workspace/h1-humanoid-control/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select h1_controller
```

Should finish in a few seconds with one package built.

Install Python dependencies needed by the controllers and plot script:

```bash
sudo apt install -y python3-yaml python3-matplotlib python3-pynput
```

Verify the package built:

```bash
source install/setup.bash
ros2 pkg list | grep h1_controller
```

Should print `h1_controller`.

---

## 6. Run the Bridge

The bridge launches Isaac Sim, loads the H1 robot, and creates an OmniGraph that exposes ROS 2 topics.

```bash
cd ~/workspace/h1-humanoid-control
./scripts/run_bridge.sh
```

First-time launch downloads NVIDIA Nucleus assets (~5 GB). This may take several minutes; the window may appear frozen during downloads.

When ready, you'll see:

- Isaac Sim window with the H1 standing on a grid plane
- Terminal log: `[bridge 2.C] Full bridge active`

Keep this terminal open.

Verify ROS 2 topics from a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

Should include `/h1/cmd_vel`, `/h1/odom`, `/clock`.

---

## 7. Drive the Robot

### Manual control (teleop)

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/h1-humanoid-control/ros2_ws/install/setup.bash
ros2 run h1_controller teleop_node
```

Keys: `W/Up` forward, `A/Left` turn left, `D/Right` turn right, `S/Space` stop, `Q/Esc` quit.

### Autonomous waypoint navigation

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/h1-humanoid-control/ros2_ws/install/setup.bash
ros2 run h1_controller waypoint_controller --ros-args \
  -p waypoints_file:=$HOME/workspace/h1-humanoid-control/config/waypoints.yaml
```

The robot walks a 2 m square back to the origin and stops.

### Watching state transitions

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /h1/waypoint_status
```

Prints `ROTATING`, `WALKING`, `REACHED`, `COMPLETE` as the controller progresses.

---

## 8. Logging and Plotting a Run

### Record a run

Four terminals, start in this order:

1. Bridge (`./scripts/run_bridge.sh`)
2. Path logger
3. Waypoint controller
4. Optionally, `ros2 topic echo /h1/waypoint_status` to watch transitions

Path logger:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/h1-humanoid-control/ros2_ws/install/setup.bash
LOG=/tmp/h1_path_$(date +%Y%m%d_%H%M%S).csv
ros2 run h1_controller path_logger --ros-args -p output:=$LOG
echo "Logging to $LOG"
```

After the controller finishes (`All waypoints reached. Stopping.`), Ctrl+C the logger to flush.

### Generate the validation plot

```bash
cd ~/workspace/h1-humanoid-control
python3 scripts/plot_path.py \
  --csv $(ls -t /tmp/h1_path_*.csv | head -1) \
  --waypoints config/waypoints.yaml \
  --out docs/waypoint_validation.png
xdg-open docs/waypoint_validation.png
```

---

## Troubleshooting

### `rclpy` import fails inside Isaac Sim

Don't try to `import rclpy` from a standalone Isaac Sim script. Isaac Sim 5.1 ships Python 3.11 and system ROS 2 Jazzy is built for Python 3.12; the bundled rclpy is unstable from standalone scripts. This project deliberately uses OmniGraph for all ROS publish/subscribe inside Isaac Sim. See [`architecture.md`](architecture.md) for the full explanation.

### `ros2 run h1_controller <node>` says `No executable found`

The package needs to be built and sourced:

```bash
cd ~/workspace/h1-humanoid-control/ros2_ws
colcon build --packages-select h1_controller
source install/setup.bash
```

### Robot motion looks too fast

The bridge enables Kit's rate limiter to keep sim time aligned with wall time. If your machine has plenty of headroom, sim time can still run faster than wall time without it. Check `/clock` rate — it should hover around 25 Hz:

```bash
ros2 topic hz /clock
```

### H1 falls during navigation

The flat-terrain policy has a hard limit of 0.75 m/s linear velocity. The waypoint controller is configured well under this limit (0.5 m/s default). If the robot still falls, try reducing the parameter:

```bash
ros2 run h1_controller waypoint_controller --ros-args -p kp_lin:=0.4 ...
```

### Permissions error on `run_bridge.sh`

```bash
chmod +x ~/workspace/h1-humanoid-control/scripts/run_bridge.sh
```

---

## What You've Just Set Up

Two independent processes communicating only via ROS 2 topics over DDS:

- **Isaac Sim** runs the H1 policy and bridges to ROS via OmniGraph
- **ROS 2 controllers** run as normal Python rclpy nodes (teleop, waypoint, logger)

This mirrors the architecture of a real humanoid deployment: the onboard locomotion controller exposes `/cmd_vel` and `/odom`, and higher-level planners talk to it over standard ROS topics. The simulator is a swap-in replacement for the real robot.

See [`architecture.md`](architecture.md) for the full design rationale.
