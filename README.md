# UR10e Force-Controlled Bowl Cleaning — ROS2 Workspace

A ROS2 Humble workspace for force-compliant robotic manipulation using a Universal Robots UR10e arm. The primary task is **force-controlled circular path following on a curved surface** (a bowl), originally developed for ISRO. The robot maintains a target contact force through a PI-controlled feedback loop while executing a circular sweep, followed by a helical retract.

---

## Table of Contents

- [About](#about)
- [Repository Structure](#repository-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Docker Workflow](#docker-workflow)
- [Configuration](#configuration)
- [Run Instructions](#run-instructions)
- [ROS2 APIs](#ros2-apis)
- [Controllers](#controllers)
- [References](#references)

---

## About

This workspace implements a force-compliant surface-following skill on a UR10e arm. The robot:

1. **Approaches** a curved bowl surface and makes contact.
2. **Follows a circular path** on the surface while a PI controller adjusts the out-of-plane displacement to maintain a constant target force (via `cartesian_compliance_controller`).
3. **Retracts helically** off the surface.

The simulation uses **Gazebo Classic** with a physically modelled bowl mesh, textured OBJ model, and a Gazebo Classic force/torque sensor (wrist) for real-time wrench feedback.

---

## Repository Structure

```
ur10e_force_control_ws/
├── Dockerfile                        # ROS2 Humble image (non-root, X11, all deps)
├── build.sh                          # Docker image build wrapper
├── docker_scripts/run_scripts/
│   ├── enter_bash.sh                 # Start container + open shell
│   └── stop.sh                       # Stop + remove container
└── workspace/
    ├── application/                  # Custom project packages
    │   ├── ur10e_simulation_pkg/     # Gazebo Classic sim, circle/retract nodes, bowl model
    │   ├── ur10e_robot_driver/       # Physical UR10e hardware driver launch
    │   ├── ur10e_testing_pkg/        # Homing, force filter, marker, controller configs
    │   └── robot_task_interfaces/    # Reserved for future custom msgs/srvs
    └── hardware/                     # Third-party submodules
        ├── Universal_Robots_ROS2_Description/    # UR URDF + meshes (fork)
        ├── Universal_Robots_ROS2_Driver/         # UR hardware driver (fork)
        ├── Universal_Robots_ROS2_Gazebo_Simulation/  # Gazebo Classic sim (fork)
        ├── Universal_Robots_ROS2_GZ_Simulation/  # GZ Fortress sim (upstream)
        └── cartesian_controllers/                # Cartesian compliance/force/motion
```

### Application Packages

| Package | Purpose |
|---|---|
| `ur10e_simulation_pkg` | Gazebo Classic launch, `ur10e_circle_follow`, `ur10e_retract`, `ur10e_reverse_circle` nodes, bowl SDF/OBJ model |
| `ur10e_robot_driver` | ROS2 launch for physical UR10e (wraps official driver) |
| `ur10e_testing_pkg` | Homing node, force filter, marker trail, `controller_manager.yaml` |
| `robot_task_interfaces` | Empty shell package (reserved for future actions/services) |

### Hardware Submodules

| Submodule | Role |
|---|---|
| `Universal_Robots_ROS2_Description` | UR URDF/xacro + custom end-effector link + Gazebo Classic FT sensor plugin |
| `Universal_Robots_ROS2_Driver` | Official UR ROS2 hardware driver |
| `Universal_Robots_ROS2_Gazebo_Simulation` | Gazebo Classic launch chain (`ur_sim_control`, `ur_sim_moveit`) |
| `cartesian_controllers` | Base, motion, force, compliance, utilities, handles sub-packages |

---

## Dependencies

| Dependency | Version |
|---|---|
| ROS2 | Humble Hawksbill |
| Gazebo | Classic (gazebo 11) |
| MoveIt2 | humble |
| Docker | 20.10+ (for containerised workflow) |

Key ROS packages installed in the Docker image:

```
ros-humble-rviz2
ros-humble-ur-robot-driver
ros-humble-ur-controllers
ros-humble-ur-moveit-config
ros-humble-ur-description
ros-humble-gazebo-ros2-control
ros-humble-cartesian-controllers   # replaced by submodule build
ros-humble-moveit-servo
ros-humble-controller-manager
ros-humble-joint-state-broadcaster
ros-humble-joint-trajectory-controller
```

---

## Installation

### Prerequisites

- Docker installed on a Linux host with an NVIDIA or Mesa GPU
- X11 display server running (for Gazebo and RViz GUI)
- Git with submodule support

### 1. Clone with submodules

```bash
git clone --recurse-submodules https://github.com/Nithish-Kumar19181/ur10e_force_control_ws.git
cd ur10e_force_control_ws
```

If you cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

### 2. Build the Docker image

```bash
./build.sh
```

This builds the image, runs `rosdep install`, and `colcon build` inside — the image ships pre-built. Submodules **must** be initialized before running this.

### 3. (Alternative) Build natively inside the container

```bash
./docker_scripts/run_scripts/enter_bash.sh   # starts container + opens shell

# Inside container:
cd /ur10e_ws
colcon build --symlink-install \
  --packages-skip cartesian_controller_simulation cartesian_controller_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For a single-package rebuild:

```bash
colcon build --symlink-install --packages-select ur10e_simulation_pkg
```

---

## Docker Workflow

| Command | What it does |
|---|---|
| `./build.sh` | Build Docker image (pre-builds workspace inside) |
| `./docker_scripts/run_scripts/enter_bash.sh` | Start container + open interactive shell |
| `./docker_scripts/run_scripts/stop.sh` | Stop and remove container |

The container mounts `workspace/` at `/ur10e_ws/src` so edits on the host are immediately visible inside (with symlink-install, no rebuild needed for Python/launch/YAML/URDF changes).

---

## Configuration

### Initial joint positions

`workspace/hardware/Universal_Robots_ROS2_Description/config/initial_positions.yaml`

Joints are set to a "home" pose where the robot is positioned above the bowl (mounted upside-down at z = 1.48 m):

```yaml
shoulder_pan_joint:   0.409
shoulder_lift_joint: -1.675
elbow_joint:         -2.402
wrist_1_joint:        0.966
wrist_2_joint:        1.165
wrist_3_joint:        3.158
```

### Spawn pose

The robot spawns upside-down above the bowl:

```
-x 0.0  -y 0.0  -z 1.48  -R 3.14159  -P 0.0  -Y 0.0
```

Configured in `workspace/hardware/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/launch/ur_sim_control.launch.py`.

### Controllers

`workspace/application/ur10e_testing_pkg/config/controller_manager.yaml`

Key controllers:

| Controller | Type | Use |
|---|---|---|
| `joint_state_broadcaster` | Always active | Publishes `/joint_states` |
| `joint_trajectory_controller` | Active at startup | Standard MoveIt motion |
| `cartesian_compliance_controller` | Inactive at startup | Force-compliant following (main task) |
| `cartesian_motion_controller` | Inactive at startup | Pure Cartesian pose tracking |
| `cartesian_force_controller` | Inactive at startup | Direct force control |

Switch controllers at runtime:

```bash
ros2 control switch_controllers \
  --activate cartesian_compliance_controller \
  --deactivate joint_trajectory_controller
```

### Cartesian compliance tuning

In `controller_manager.yaml` under `cartesian_compliance_controller`:

```yaml
stiffness:
  trans_x: 2900.0
  trans_y: 2900.0
  trans_z: 200.0    # low Z stiffness → compliant in contact direction
  rot_x: 1500.0
  rot_y: 1500.0
  rot_z: 1500.0
```

### Bowl model

`workspace/application/ur10e_simulation_pkg/model/`

The bowl SDF is templated — `PACKAGE_PATH` is substituted at launch time with the installed share path, so all `file://` URIs resolve correctly regardless of install location.

---

## Run Instructions

### 1. Start Gazebo Classic simulation (full stack)

```bash
# Inside container or sourced native install:
ros2 launch ur10e_simulation_pkg robot_sim.launch.py
```

This starts:
- Gazebo Classic + bowl model
- `robot_state_publisher`
- MoveIt2 (`move_group`, `servo_node`)
- RViz2 with custom config
- `joint_state_broadcaster` + `joint_trajectory_controller`
- `cartesian_motion/force/compliance_controller` (inactive, ready to switch)
- `ur10e_force_filter` (low-pass filter: `/wrench` → `/cartesian_compliance_controller/ft_sensor_wrench`)
- `ur10e_marker_array` (tool path trail in RViz)

> **Note:** Gazebo Classic takes ~60 seconds after the robot spawns before controllers become available. The TF warnings seen during this period are expected and resolve once `joint_state_broadcaster` activates.

### 2. Home the robot

```bash
ros2 run ur10e_testing_pkg ur10e_home
```

### 3. Run the circle-following task

Switch to the compliance controller first:

```bash
ros2 control switch_controllers \
  --activate cartesian_compliance_controller \
  --deactivate joint_trajectory_controller
```

Then run the node:

```bash
ros2 run ur10e_simulation_pkg ur10e_circle_follow
```

### 4. Run the helical retract

```bash
ros2 run ur10e_simulation_pkg ur10e_retract
```

### 5. Physical robot (hardware)

```bash
ros2 launch ur10e_robot_driver robot_driver.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

---

## ROS2 APIs

### Key topics

| Topic | Type | Description |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions/velocities from broadcaster |
| `/wrench` | `geometry_msgs/WrenchStamped` | Raw FT sensor from Gazebo Classic (wrist_3_joint) |
| `/cartesian_compliance_controller/ft_sensor_wrench` | `geometry_msgs/WrenchStamped` | Low-pass filtered wrench fed to compliance controller |
| `/cartesian_compliance_controller/target_frame` | `geometry_msgs/PoseStamped` | Cartesian setpoint for compliance controller |
| `/tool_path_markers` | `visualization_msgs/MarkerArray` | Tool tip trail for RViz |

### Controller switching service

```bash
ros2 control switch_controllers --activate <name> --deactivate <name>
ros2 control list_controllers
```

### Force filter node

`ur10e_force_filter` subscribes to `/wrench` and publishes a low-pass filtered wrench to `/cartesian_compliance_controller/ft_sensor_wrench`. The filter coefficient `alpha` is set in the node source.

For MoveIt2 planning and execution APIs, refer to the [MoveIt2 documentation](https://moveit.picknik.ai/humble/index.html). For cartesian controller parameters, see the [cartesian_controllers wiki](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).

---

## Controllers

All controllers are defined in `ur10e_testing_pkg/config/controller_manager.yaml`. The full list registered with the controller manager:

```
joint_state_broadcaster          (always active)
joint_trajectory_controller      (active at startup — used with MoveIt)
scaled_joint_trajectory_controller
cartesian_compliance_controller  (inactive — main force-control task)
cartesian_motion_controller      (inactive)
cartesian_force_controller       (inactive)
forward_velocity_controller
forward_position_controller
force_torque_sensor_broadcaster
io_and_status_controller
speed_scaling_state_broadcaster
```

---

## References

- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [Cartesian Controllers (FZI)](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
- [MoveIt2 — Humble](https://moveit.picknik.ai/humble/index.html)
- [gazebo_ros2_control](https://github.com/ros-controls/gazebo_ros2_control)
- [ROS2 Control Documentation](https://control.ros.org/humble/index.html)
