# UR10e Robotic Arm Control and Simulation for ISRO

This repository contains a complete ROS2 workspace for controlling and simulating a Universal Robots UR10e arm. It includes packages for simulation in Gazebo, hardware drivers, advanced Cartesian controllers, and a custom desktop GUI for task management and monitoring. The primary application demonstrated is a force-compliant circular following task on a curved surface.

## Key Features

- **ROS2 Integration:** Built entirely on ROS2 for modern, robust robotic applications.
- **Gazebo Simulation:** A complete simulation environment using `ros_gz_sim` (Gazebo Ignition) with a UR10e robot and a target object (bowl) for interaction.
- **Hardware & Simulation Launchers:** Separate launch files for running with either `ur_robot_driver` for a physical robot or a simulated one.
- **Advanced Control:** Implements and allows switching between `joint_trajectory_controller` for standard motions and `cartesian_compliance_controller` for force-sensitive tasks.
- **Desktop Control GUI:** A PyQt5-based graphical user interface to:
    - Launch and terminate the simulation and individual robot tasks.
    - Switch between Joint Trajectory and Cartesian Compliance controllers.
    - Visualize real-time force/torque sensor data from the end-effector.
    - Monitor the status of running processes.
- **Task Sequencing with Action Servers:** A modular system for defining and executing complex tasks.
    - `robot_task_interfaces`: Defines a custom `ExecuteSkill` action.
    - Action servers for skills like "Approach and Circle" and "Helical Retract".
    - A `task_manager` node that acts as a client to sequence these skills.
- **Force-Compliant Motion:** The `action_circle_follow` node uses a PI controller on force sensor feedback to dynamically adjust the robot's path radius, ensuring constant contact with a surface.

## Repository Structure

This workspace is organized into several ROS2 packages:

-   `robot_desktop_gui`: Provides a PyQt5-based GUI for high-level control and visualization.
-   `ur10e_simulation_pkg`: Contains launch files, nodes, and models for the Gazebo simulation environment.
-   `ur10e_robot_driver`: Includes launch files and nodes for controlling a physical or fake (driver-level) UR10e robot.
-   `ur10e_testing_pkg`: A utility package with controller configurations (`controller_manager.yaml`), robot homing scripts, and sensor data processing nodes.
-   `ur10e_vision`: RGB-D surface-cleaning **path generation** pipeline for the mixer blades — takes the full organized D435i depth frame, patches interior holes, estimates surface normals, and emits a raster (lawnmower) cleaning path as `base_link` waypoints (per-waypoint TF + RViz markers). See [Surface-Cleaning Vision Pipeline](#surface-cleaning-vision-pipeline-ur10e_vision).
-   `robot_task_interfaces`: Defines the custom ROS2 `ExecuteSkill.action` used for standardized task execution.
-   **Dependencies (Submodules):**
    - `Universal_Robots_ROS2_Driver`: The official ROS2 driver for UR robots.
    - `Universal_Robots_ROS2_Description`: URDF models for the robots.
    - `Universal_Robots_ROS2_GZ_Simulation`: Simulation-specific resources for Gazebo.
    - `cartesian_controllers`: The ROS2 control stack for Cartesian motion, force, and compliance control.

---

## How the Gazebo Model Moves

### Overview
The robot movement in Gazebo is enabled through a **ROS2 control framework** that bridges simulation physics with joint control commands. Here's how it works:

### Key Components

**1. URDF with ros2_control Tags**
- The robot is defined in `Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro`
- When launched with `sim_gazebo:=true`, the URDF includes the `gazebo_ros2_control` plugin
- This tells Gazebo how to treat the robot model and what interfaces are available

**2. Gazebo ROS2 Control Plugin** (`libgazebo_ros2_control.so`)
- Acts as a bridge between ROS2 controllers and Gazebo's physics engine
- Reads control commands from ROS2 topics
- Converts them to forces/torques applied to joints in Gazebo
- Updates joint positions/velocities based on physics simulation

**3. Controller Manager**
- Central node that manages all control plugins
- Runs at configurable rate (100 Hz by default as per `ur_controllers.yaml`)
- Spawned during simulation startup and communicates with the Gazebo plugin

**4. Controllers** (defined in [ur_simulation_gazebo/config/ur_controllers.yaml](src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/config/ur_controllers.yaml))
- `joint_trajectory_controller`: Receives trajectory commands and generates joint position references
- `scaled_joint_trajectory_controller`: Speed-scaled variant for safety-limited motion
- `forward_velocity_controller`: Direct velocity control
- `forward_position_controller`: Direct position control
- `joint_state_broadcaster`: Continuously publishes current joint states (positions/velocities)

### Movement Flow

```
User Command (ROS2 topic)
    ↓
Joint Trajectory Controller (receives & parses command)
    ↓
Gazebo ROS2 Control Plugin (converts to joint forces)
    ↓
Gazebo Physics Engine (simulates joint motion based on forces & inertia)
    ↓
Joint State Broadcaster (publishes updated joint positions)
    ↓
RViz/Controllers (receive feedback & visualize)
```

### Launch Sequence
The [ur_sim_control.launch.py](src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/launch/ur_sim_control.launch.py) file orchestrates the startup:
1. Generates robot URDF with Gazebo plugin enabled
2. Launches Gazebo with physics enabled
3. Spawns the robot model into Gazebo
4. Starts the Controller Manager
5. Spawns the Joint State Broadcaster (publishes monitored states)
6. Spawns the Joint Trajectory Controller (ready to receive commands)

### Why This Architecture?
- **Decoupled Control**: Controllers don't directly manipulate Gazebo; they publish standard ROS2 interface commands
- **Real-like Simulation**: Gazebo realistically simulates physics, inertia, gravity, and joint friction
- **Easy Hardware Transfer**: The same controller configuration works with real hardware by simply loading a different hardware interface
- **Extensible**: New controllers can be added without modifying the core simulation

---

## Installation and Setup

### Prerequisites

-   ROS 2 (Humble Hawksbill recommended)
-   Gazebo (Ignition Fortress) and the `ros_gz_sim` bridge.
-   MoveIt 2 for ROS 2.
-   Colcon build tools.

### Building the Workspace

1.  **Clone the repository:** Clone this repository and its submodules into your ROS2 workspace's `src` directory.

    ```bash
    git clone --recurse-submodules https://github.com/Nithish-Kumar19181/Robotic_arm_ISRO.git
    ```

2.  **Install Dependencies:** Use `rosdep` to install all necessary package dependencies.

    ```bash
    cd <your_ros2_workspace>
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the Workspace:** Compile all the packages using `colcon`.

    ```bash
    colcon build --symlink-install
    ```

4.  **Source the Workspace:** In every new terminal, source the setup file to make the packages available.

    ```bash
    source install/setup.bash
    ```

---

## Usage

You can run the project either in simulation or with a physical robot. The Control GUI is the recommended way to interact with the system.

### 1. Launch the Simulation or Robot Driver

First, launch either the Gazebo simulation or the hardware driver.

**Option A: Launch Gazebo Simulation**

This will start Gazebo, spawn the UR10e, and a bowl model. It also launches MoveIt and the necessary controllers.

```bash
ros2 launch ur10e_simulation_pkg robot_sim.launch.py
```

**Option B: Launch Hardware Driver**

This connects to a physical UR10e robot. Make sure to set the correct `robot_ip`. For testing without hardware, you can use `use_fake_hardware:=true`.

```bash
ros2 launch ur10e_robot_driver robot_driver.launch.py robot_ip:=<your_robot_ip> use_fake_hardware:=true
```

### 2. Launch the Control GUI

In a new sourced terminal, run the GUI node.

```bash
ros2 run robot_desktop_gui gui_node
```

The GUI provides buttons to:

-   **Launch Simulation:** A shortcut to run the `robot_sim.launch.py` file.
-   **Home Robot:** Moves the robot to a predefined home position using MoveIt.
-   **Activate Controllers:** Switch between `Joint Trajectory` (for homing) and `Cartesian Compliance` (for force-based tasks).
-   **Run Tasks:** Execute the `Circle Follow` or `Retract` skills.
-   **Task Manager:** Runs a node that sequentially executes the circle follow and retract skills in a loop.
-   **Stop All Processes:** An emergency stop that terminates all managed subprocesses.

### 3. Running Individual Nodes

You can also run individual task nodes directly from the command line after the simulation/driver is active.

-   **Move to Home Position:**

    ```bash
    ros2 run ur10e_testing_pkg ur10e_home
    ```

-   **Execute the Circle Following Task:**
    *First, ensure the Cartesian Compliance Controller is active.* You can do this via the GUI or with the command:
    ```bash
    ros2 control switch_controllers --activate cartesian_compliance_controller --deactivate joint_trajectory_controller
    ```
    Then, run the action server node:
    ```bash
    ros2 run ur10e_simulation_pkg action_circle_follow
    ```

-   **Execute the Retract Task:**
    ```bash
    ros2 run ur10e_simulation_pkg action_retract
    ```

-   **Visualize Tool Path in RViz:**
    A marker array node is included in the simulation launch file to publish a trail of the `tool0` link's path. Open RViz and add the `/tool_marker_array` topic of type `visualization_msgs/msg/MarkerArray` to see the trail.

---

## Surface-Cleaning Vision Pipeline (`ur10e_vision`)

Generates a **surface-following cleaning path** on the vertical-mixer blades from the
wrist-mounted Intel RealSense **D435i** RGB-D camera. The output is a set of
surface-constrained waypoints (3D position + normal-aligned orientation), published as
per-waypoint TF frames and RViz markers — ready to be consumed later by a
motion-planning system. **No robot motion / IK / MoveIt execution is involved at this
stage** — it is purely perception → path generation.

Both nodes are C++/PCL; there is no Python, Open3D or CAD dependency. The capture is
**one-shot on a service trigger**, then continuously re-streamed so late-joining
subscribers (RViz, downstream nodes) always receive the last result.

### Pipeline overview

```
D435i depth (/camera/camera/points, organized 640x480)
   │
   ▼  Node A: cloud_processor  (C++ / PCL)
   │   • one-shot capture on /ur10e_vision/trigger (full frame, no crop)
   │   • depth-band passthrough (keeps the cloud organized, holes as NaN)
   │   • interior hole fill: bilinear, gap-capped (max_fill_gap_px)
   │   • transform into base_link
   │   • store snapshot; a timer re-streams it at republish_rate_hz
   │   → /ur10e_vision/surface_cloud  (latched)  +  /ur10e_vision/roi
   ▼  Node B: waypoint_generator  (C++)
   │   • raster grid over the organized cloud (metres → pixels via median step)
   │   • window = raster_length × raster_height, slid by raster_center_x/y
   │   • fill grid gaps (linear interp + edge extension) + Catmull-Rom smoothing
   │   • normals: cross product of along-/across-pass tangents, oriented to camera
   │   • orientation: +Z = approach_axis_sign · normal, +X = path tangent
   │   • hover by standoff along the normal; serpentine pass order
   │   • only regenerates on a new capture (dedupes re-streamed clouds)
   ▼
/ur10e_vision/path_markers   (MarkerArray: path line + normal arrows)
TF: clean_wp_000 … clean_wp_NNN
```

> The waypoint `PoseArray` is built internally to derive the markers and TF, but is
> **not published** on a topic — the RViz markers + TF frames are the consumable output.

### Prerequisites

Both nodes depend only on **PCL** (`libpcl-all-dev`) and the usual `tf2`/`pcl_ros`
packages — all pulled in by `rosdep install` (see [Building the Workspace](#building-the-workspace)).

### Build

```bash
colcon build --symlink-install --packages-select ur10e_vision
source install/setup.bash
```

### Run

1. **Launch the mixer scene** (provides the arm, the D435i camera and the TF tree).
   Pose the arm so a blade is within the camera's view:

   ```bash
   ros2 launch ur10e_simulation_pkg vertical_mixer_ur10e.launch.py
   ```

2. **Start the vision pipeline** (both nodes). Add `rviz:=true` for a ready-made view:

   ```bash
   ros2 launch ur10e_vision cleaning_path.launch.py
   # or with the bundled RViz config:
   ros2 launch ur10e_vision cleaning_path.launch.py rviz:=true
   ```

3. **Trigger a one-shot capture + path generation:**

   ```bash
   ros2 service call /ur10e_vision/trigger std_srvs/srv/Trigger {}
   ```

   Expected logs:
   - `cloud_processor`: `Hole fill: patched N interior cells …` → `Published surface_cloud … ROI […]`
   - `waypoint_generator`: `Generated K waypoints (pitch=…px/0.05m along=…px/0.04m, gaps bridged)`

   Re-trigger any time to recompute against the current scene; between triggers the last
   cloud keeps streaming and the path is **not** recomputed.

### Inspect the output

```bash
ros2 run tf2_ros tf2_echo base_link clean_wp_000       # first waypoint frame
ros2 topic hz /ur10e_vision/surface_cloud              # ~republish_rate_hz while idle
```

In RViz (Fixed Frame = `base_link`) add, all with **Durability = Transient Local**:
`/ur10e_vision/surface_cloud` (PointCloud2), `/ur10e_vision/path_markers` (MarkerArray)
and `TF`.

### Key parameters (`ur10e_vision/config/vision_params.yaml`)

| Parameter | Node | Meaning | Default |
|---|---|---|---|
| `input_cloud_topic` | A | Organized depth cloud topic | `/camera/camera/points` |
| `depth_min` / `depth_max` | A | Depth-band passthrough (m) | `0.15` / `4.0` |
| `max_fill_gap_px` | A | Widest interior hole (px) to bridge | `40` |
| `republish_rate_hz` | A | Re-stream rate of the stored cloud (0 = off) | `1.0` |
| `output_frame` | A & B | Frame for cloud & waypoints | `base_link` |
| `raster_direction` | B | `horizontal` (passes step vertically) or `vertical` | `horizontal` |
| `tool_width` | B | Pass-to-pass pitch (m) | `0.05` |
| `waypoint_spacing` | B | Along-pass spacing (m) | `0.04` |
| `standoff` | B | Offset along outward normal (m) | `0.07` |
| `raster_length` / `raster_height` | B | Raster window size (m) | `0.70` / `0.55` |
| `raster_center_x` / `raster_center_y` | B | Slide window in image plane (m) | `0.0` / `0.2` |
| `approach_axis_sign` | B | `-1` = tool +Z into surface | `-1.0` |

### Troubleshooting

- **`No point cloud received yet`** on trigger — the camera isn't publishing; check
  `ros2 topic hz /camera/camera/points` and that the mixer scene is running.
- **`No valid points after depth filtering`** — the surface isn't in view or is outside
  the depth band; re-pose the arm or widen `depth_min`/`depth_max`.
- **Holes remain in the surface cloud** — raise `max_fill_gap_px` (only *interior* NaN
  runs narrower than this are bridged; the outer silhouette is always kept).
- **`raster_length/raster_height too small for a grid`** — the window yields <2 passes;
  increase them or re-center with `raster_center_x/y`.
- **`No TF base_link <- camera_*_optical_frame`** — the camera TF isn't being published
  (normals fall back to camera-facing); make sure the mixer scene with the D435i is up.
