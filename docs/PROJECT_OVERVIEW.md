# ROS 2 Multi-Robot Simulation Project — Full Documentation

> **Workspace**: `/ros2_ws_multirobots`  
> **ROS 2 Distribution**: Humble / Iron (ament_cmake build system)  
> **Simulator**: Gazebo Sim (Ignition Gazebo, `gz-sim`)  
> **Date**: March 2026

---

## Table of Contents

1. [Project Goal](#1-project-goal)
2. [High-Level Architecture](#2-high-level-architecture)
3. [Workspace Layout](#3-workspace-layout)
4. [Package Descriptions](#4-package-descriptions)
   - 4.1 [`practica1` — Robot Description](#41-practica1--robot-description)
   - 4.2 [`robot_bringup` — Orchestration & Launch](#42-robot_bringup--orchestration--launch)
5. [How to Build & Run](#5-how-to-build--run)
6. [Controlling the Robots](#6-controlling-the-robots)
7. [TF Tree & Frames](#7-tf-tree--frames)
8. [Key Design Decisions](#8-key-design-decisions)

---

## 1. Project Goal

Spawn **N identical differential-drive robots** inside a single Gazebo Sim world and expose every robot's topics (velocity commands, odometry, laser scan, camera image, joint states, TF) into ROS 2 under isolated namespaces (`/robot1`, `/robot2`, …, `/robotN`).

The number of robots is configurable at launch time via a single argument — no source code changes are required when scaling from 1 to *N* robots.

---

## 2. High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Gazebo Sim                                │
│  ┌────────┐  ┌────────┐       ┌────────┐                        │
│  │ robot1 │  │ robot2 │  ...  │ robotN │    my_world.sdf        │
│  └───┬────┘  └───┬────┘       └───┬────┘                        │
│      │  gz topics │                │                             │
└──────┼────────────┼────────────────┼─────────────────────────────┘
       │            │                │
       ▼            ▼                ▼
┌──────────────────────────────────────────────────────────────────┐
│                  ros_gz_bridge  (single node)                    │
│   Dynamically-generated YAML config maps every robot's topics   │
│   /robotX/cmd_vel  ←→  robotX/cmd_vel   (ROS ↔ GZ)             │
│   /robotX/odom     ←   robotX/odom      (GZ → ROS)             │
│   /robotX/scan     ←   robotX/scan      (GZ → ROS)             │
│   /robotX/camera/* ←   robotX/camera/*  (GZ → ROS)             │
│   /robotX/joint_states ← ...            (GZ → ROS)             │
│   /tf              ←   robotX/tf        (GZ → ROS)             │
│   /clock           ←   /clock           (GZ → ROS)             │
└──────────────────────────────────────────────────────────────────┘
       │            │                │
       ▼            ▼                ▼
┌──────────────────────────────────────────────────────────────────┐
│                      ROS 2 Graph                                 │
│                                                                  │
│  robot_state_publisher (×N, namespaced)                          │
│  static_transform_publisher (×N)   world → robotX/odom          │
│                                                                  │
│  User nodes: teleop, Nav2, custom controllers …                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. Workspace Layout

```
ros2_ws_multirobots/
├── src/
│   ├── practica1/                   # Robot description package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch/
│   │   │   └── sencillo_launch.launch   # Simple single-robot RViz launch (XML)
│   │   └── urdf/
│   │       ├── robotito.urdf.xacro      # Main robot model (parametrised)
│   │       ├── robotito.urdf            # Static URDF (single robot, no ns)
│   │       ├── gazebo_control.xacro     # Diff-drive + joint-state Gazebo plugins
│   │       ├── lidar.xacro             # GPU LiDAR sensor definition
│   │       ├── camera.xacro            # RGB camera sensor definition
│   │       └── inertial_macros.xacro   # Reusable inertia calculation macros
│   │
│   └── robot_bringup/               # Launch & orchestration package
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── robot_bringup.launch.py  # Main multi-robot launch (Python)
│       ├── config/
│       │   └── bridge_config.yaml       # Static bridge config (reference/legacy)
│       └── world/
│           └── my_world.sdf            # Gazebo world file
│
├── build/       # colcon build artefacts
├── install/     # colcon install space
├── log/         # colcon build logs
├── frames_2026-02-04_17.50.44.gv   # TF tree snapshot (Graphviz)
└── README.md
```

---

## 4. Package Descriptions

### 4.1 `practica1` — Robot Description

This package owns the robot's URDF/Xacro model. It has **no C++ or Python nodes** — it only installs static assets (URDF files and a legacy launch file).

#### 4.1.1 `robotito.urdf.xacro` — Main Robot Model

| Aspect | Detail |
|--------|--------|
| **Robot name** | `robotito` |
| **Drive type** | Differential drive (2 powered wheels + 1 passive caster) |
| **Sensors** | GPU LiDAR (360°, 12 m range) + RGB camera (800×800 @ 30 fps) |

**Namespace mechanism** — The file declares a `prefix` argument:

```xml
<xacro:arg name="prefix" default="" />
<xacro:property name="ns_prefix" value="$(arg prefix)" />
```

Every link and joint name is prepended with `${ns_prefix}`. When the launch file processes this xacro with `mappings={'prefix': 'robot1/'}`, all names become `robot1/base_link`, `robot1/left_wheel_joint`, etc. This is the core trick that allows multiple identical robots to coexist without name collisions.

**Included sub-files:**

| File | Purpose |
|------|---------|
| `inertial_macros.xacro` | Provides `inertial_sphere`, `inertial_box`, `inertial_cylinder` macros |
| `gazebo_control.xacro` | Attaches Gazebo diff-drive and joint-state-publisher plugins |
| `lidar.xacro` | Defines the LiDAR sensor link, joint, and Gazebo `<sensor>` |
| `camera.xacro` | Defines the camera mast, camera link, optical frame, and Gazebo `<sensor>` |

**Physical structure:**

```
base_link  (green box 0.4×0.3×0.1 m, 1 kg)
├── left_wheel_joint  (continuous) → left_wheel  (cylinder r=0.1, w=0.04)
├── right_wheel_joint (continuous) → right_wheel (cylinder r=0.1, w=0.04)
├── caster_joint (fixed) → caster_wheel (sphere r=0.025, μ=0.1)
├── laser_joint (fixed) → laser_frame  (LiDAR at z=0.4 m)
└── camera_mast_joint (fixed) → camera_mast (cylinder)
    └── camera_joint (fixed) → camera_link (blue box)
        └── camera_optical_joint (fixed) → camera_link_optical
```

---

#### 4.1.2 `inertial_macros.xacro` — Inertia Helpers

Provides three xacro macros that automatically compute the inertia tensor from mass and shape parameters, using standard physics formulae:

| Macro | Parameters | Formula basis |
|-------|-----------|---------------|
| `inertial_sphere` | `mass`, `radius` | $I = \frac{2}{5} m r^2$ |
| `inertial_box` | `mass`, `x`, `y`, `z` | $I_{xx} = \frac{1}{12} m (y^2 + z^2)$, etc. |
| `inertial_cylinder` | `mass`, `length`, `radius` | $I_{xx} = \frac{1}{12} m (3r^2 + l^2)$, $I_{zz} = \frac{1}{2} m r^2$ |

These are called throughout the main xacro file using the form:
```xml
<xacro:inertial_box mass="0.5" x="0.4" y="0.3" z="0.1">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:inertial_box>
```

---

#### 4.1.3 `gazebo_control.xacro` — Gazebo Simulation Plugins

This file attaches three Gazebo-specific elements to the robot model:

**1. Caster wheel friction override**
```xml
<gazebo reference="$(arg prefix)caster_wheel">
    <mu1 value="0.1" />
    <mu2 value="0.1" />
</gazebo>
```
Sets very low friction on the caster wheel so it slides freely and doesn't resist turning.

**2. Differential Drive plugin** (`gz-sim-diff-drive-system`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `left_joint` | `{prefix}left_wheel_joint` | Left wheel joint name |
| `right_joint` | `{prefix}right_wheel_joint` | Right wheel joint name |
| `wheel_separation` | `0.35` m | Distance between wheel centres |
| `wheel_radius` | `0.1` m | Wheel radius |
| `odom_publish_frequency` | `1` Hz | Odometry publication rate |
| `topic` | `{prefix}cmd_vel` | Gazebo topic to subscribe for velocity commands |
| `odom_topic` | `{prefix}odom` | Gazebo topic to publish odometry |
| `tf_topic` | `{prefix}tf` | Gazebo topic to publish `odom → base_link` transform |
| `frame_id` | `{prefix}odom` | Odometry frame |
| `child_frame_id` | `{prefix}base_link` | Robot body frame |

**3. Joint State Publisher plugin** (`gz-sim-joint-state-publisher-system`)

Publishes the position/velocity of both wheel joints on the Gazebo topic `{prefix}joint_states`.

> **Note:** All topic names include `$(arg prefix)`, so each robot instance publishes on its own namespaced Gazebo topics.

---

#### 4.1.4 `lidar.xacro` — LiDAR Sensor

Adds a GPU LiDAR sensor mounted on top of the robot (z = 0.4 m above `base_link`).

| Property | Value |
|----------|-------|
| Type | `gpu_lidar` |
| Horizontal samples | 360 |
| FOV | 360° (−π to +π) |
| Min range | 0.3 m |
| Max range | 12 m |
| Update rate | 10 Hz |
| Noise | Gaussian, σ = 0.01 |
| Gazebo topic | `{prefix}scan` |
| Frame ID | `{prefix}laser_frame` |

---

#### 4.1.5 `camera.xacro` — RGB Camera

Adds a camera on a cylindrical mast (0.30 m tall) mounted at the front of the robot.

| Property | Value |
|----------|-------|
| Type | `camera` |
| Resolution | 800 × 800 px |
| Format | R8G8B8 |
| HFOV | 1.3963 rad (~80°) |
| Update rate | 30 Hz |
| Clip range | 0.02 – 300 m |
| Gazebo topic | `{prefix}camera/image_raw` |
| Frame ID | `{prefix}camera_link_optical` |

The `camera_optical_joint` applies a −90° roll and −90° yaw rotation to convert from the link frame (x-forward) to the optical convention (z-forward, x-right, y-down), as required by ROS image processing nodes.

---

#### 4.1.6 `sencillo_launch.launch` — Simple RViz Visualisation (XML Launch)

A minimal launch file for visualising a **single robot** in RViz (no Gazebo, no namespacing):

```xml
<launch>
    <let name="urdf_path" value="$(find-pkg-share practica1)/urdf/robotito.urdf" />
    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'cat $(var urdf_path)')" />
    </node>
    <node pkg="joint_state_publisher" exec="joint_state_publisher" />
    <node pkg="rviz2" exec="rviz2" output="screen" />
</launch>
```

It reads the **static `robotito.urdf`** file (not the xacro), starts `robot_state_publisher` + `joint_state_publisher` + RViz2. Useful for checking the robot's visual model and TF tree without running simulation.

---

#### 4.1.7 `robotito.urdf` — Static URDF (Single Robot)

A pre-baked, non-parametrised URDF containing only the chassis, wheels, and caster — **no sensors and no Gazebo plugins**. This is the file used by `sencillo_launch.launch` for simple RViz visualisation.

---

### 4.2 `robot_bringup` — Orchestration & Launch

This is the **multi-robot orchestration package**. It contains no C++ or Python library code — only a launch file, a bridge config, and a world file.

#### 4.2.1 `robot_bringup.launch.py` — The Main Launch File

This is the heart of the project. It is a **Python launch file** that dynamically generates all the ROS 2 nodes needed to simulate N robots.

##### Launch Argument

| Argument | Default | Description |
|----------|---------|-------------|
| `num_robots` | `2` | Number of robot instances to spawn |

Usage:
```bash
ros2 launch robot_bringup robot_bringup.launch.py num_robots:=5
```

##### Execution Flow (step by step)

The file uses `OpaqueFunction` to defer node creation until launch arguments are resolved:

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        OpaqueFunction(function=launch_setup),
    ])
```

Inside `launch_setup(context)`:

---

**Step 1 — Start Gazebo Sim (once)**

```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': f'-r {world_file}'}.items(),
)
```

Launches Gazebo with the custom world (`my_world.sdf`) in "run" mode (`-r`).

---

**Step 2 — Calculate spawn positions**

```python
radius = 2.0
for i in range(num_robots):
    angle = 2.0 * math.pi * i / num_robots
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    spawn_positions.append((x, y))
```

Distributes robots evenly on a circle of radius 2 m centred at the origin. For example:
- 1 robot → (2, 0)
- 2 robots → (2, 0) and (−2, 0)
- 3 robots → triangle at 120° apart
- 4 robots → square at 90° apart

---

**Step 3 — Build dynamic bridge configuration**

Instead of using a static YAML file, the launch script **generates the bridge config programmatically**:

```python
bridge_entries = [
    # Global /clock (only once)
    {'ros_topic_name': '/clock', 'gz_topic_name': '/clock', ...},
]

for i in range(num_robots):
    ns = f'robot{i+1}'
    # Add per-robot entries: cmd_vel, joint_states, scan, camera, odom, tf
    bridge_entries.extend(per_robot)
```

The resulting YAML is written to a temp file (`/tmp/multi_robot_bridge_config.yaml`) and passed to a **single** `ros_gz_bridge` node:

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_yaml_path, 'use_sim_time': True}],
)
```

**Bridged topics per robot:**

| ROS 2 Topic | Gazebo Topic | Direction | Type |
|-------------|-------------|-----------|------|
| `/{ns}/cmd_vel` | `/{ns}/cmd_vel` | ROS → GZ | `geometry_msgs/Twist` |
| `/{ns}/joint_states` | `/{ns}/joint_states` | GZ → ROS | `sensor_msgs/JointState` |
| `/{ns}/scan` | `/{ns}/scan` | GZ → ROS | `sensor_msgs/LaserScan` |
| `/{ns}/camera/image_raw` | `/{ns}/camera/image_raw` | GZ → ROS | `sensor_msgs/Image` |
| `/{ns}/odom` | `/{ns}/odom` | GZ → ROS | `nav_msgs/Odometry` |
| `/tf` | `/{ns}/tf` | GZ → ROS | `tf2_msgs/TFMessage` |

> Note: All per-robot GZ-side TF topics are funnelled into the single global `/tf` topic on the ROS side.

---

**Step 4 — For each robot: Xacro → RSP → Spawn → Static TF**

For each robot `i` (namespace `robot{i+1}`):

**4a. Process Xacro**
```python
doc = xacro.process_file(xacro_file, mappings={'prefix': f'{ns}/'})
robot_desc = doc.toxml()
```
The `prefix` mapping causes every link/joint name and every Gazebo topic to be prefixed with `robot1/`, `robot2/`, etc.

**4b. Robot State Publisher** (namespaced)
```python
rsp = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=ns,
    parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
)
```
- Runs under namespace `/{ns}` so its services (`/{ns}/get_urdf`, etc.) don't clash.
- Explicitly remaps `/tf` and `/tf_static` to the global topics (otherwise they'd become `/{ns}/tf`).
- Publishes all the static transforms (e.g. `base_link → laser_frame`, `base_link → camera_mast`).

**4c. Spawn Entity**
```python
spawn = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-string', robot_desc, '-name', ns, '-x', str(x), '-y', str(y), '-z', '0.1'],
)
```
Sends the robot URDF/SDF to Gazebo with a unique entity name and initial position.

**4d. Static Transform: `world → {ns}/odom`**
```python
static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['--x', str(x), '--y', str(y), '--z', '0',
               '--frame-id', 'world', '--child-frame-id', f'{ns}/odom'],
)
```
Anchors each robot's odometry frame in the global `world` frame at the robot's spawn position. This makes it possible to visualise all robots simultaneously in RViz using a single `world` fixed frame.

---

**Summary of nodes launched per configuration:**

| Node | Count | Purpose |
|------|-------|---------|
| Gazebo Sim | 1 | Physics simulation |
| `ros_gz_bridge` | 1 | All topic bridging |
| `robot_state_publisher` | N | Publish URDF + static TFs per robot |
| `ros_gz_sim create` | N | Spawn each robot entity |
| `static_transform_publisher` | N | `world → robotX/odom` per robot |
| **Total** | 3N + 2 | |

---

#### 4.2.2 `bridge_config.yaml` — Static Bridge Config (Reference)

```yaml
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ...
```

This is a **legacy/reference** configuration for a **single robot** (no namespaces). The actual multi-robot launch bypasses this file and generates the configuration dynamically in Python. It is kept for documentation and for potential single-robot use.

---

#### 4.2.3 `my_world.sdf` — Gazebo World

A minimal Gazebo Sim world with:

| Element | Description |
|---------|-------------|
| **Physics** | 1 ms step size, real-time factor = 1.0 |
| **Physics plugin** | `gz::sim::systems::Physics` |
| **User Commands** | `gz::sim::systems::UserCommands` — allows spawning/deleting models |
| **Scene Broadcaster** | `gz::sim::systems::SceneBroadcaster` — publishes scene state |
| **Sensors** | `gz::sim::systems::Sensors` with `ogre2` render engine — processes sensors |
| **Sun** | Directional light with shadows |
| **Ground Plane** | 100×100 m flat grey plane |

The world is intentionally empty to allow spawning robots at runtime via the launch file.

---

## 5. How to Build & Run

### Prerequisites

- ROS 2 (Humble or Iron)
- Gazebo Sim (Fortress or Garden)
- Packages: `ros-<distro>-ros-gz-sim`, `ros-<distro>-ros-gz-bridge`, `ros-<distro>-robot-state-publisher`, `ros-<distro>-xacro`, `ros-<distro>-tf2-ros`

### Build

```bash
cd /ros2_ws_multirobots
colcon build --symlink-install
source install/setup.bash
```

### Launch (Multi-Robot)

```bash
# Default: 2 robots
ros2 launch robot_bringup robot_bringup.launch.py

# Custom: 5 robots
ros2 launch robot_bringup robot_bringup.launch.py num_robots:=5
```

### Launch (Single Robot — RViz only, no simulation)

```bash
ros2 launch practica1 sencillo_launch.launch
```

---

## 6. Controlling the Robots

Each robot listens on its own `cmd_vel` topic. To move a specific robot:

```bash
# Move robot1 forward
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Spin robot2 in place
ros2 topic pub /robot2/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

Using `teleop_twist_keyboard` for a specific robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

### Available topics per robot (`/robotX/...`)

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/robotX/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Velocity commands |
| `/robotX/odom` | `nav_msgs/Odometry` | Publish | Odometry (pose + twist) |
| `/robotX/scan` | `sensor_msgs/LaserScan` | Publish | 360° LiDAR scan |
| `/robotX/camera/image_raw` | `sensor_msgs/Image` | Publish | RGB camera image |
| `/robotX/joint_states` | `sensor_msgs/JointState` | Publish | Wheel joint positions/velocities |
| `/robotX/robot_description` | `std_msgs/String` | Publish | URDF XML (from RSP) |

### Global topics

| Topic | Description |
|-------|-------------|
| `/clock` | Simulation time |
| `/tf` | All dynamic transforms (merged from all robots) |
| `/tf_static` | All static transforms (merged from all robots) |

---

## 7. TF Tree & Frames

For a 2-robot configuration, the complete TF tree looks like:

```
world
├── robot1/odom  (static TF at spawn position)
│   └── robot1/base_link  (dynamic — from diff-drive odometry)
│       ├── robot1/left_wheel    (dynamic — wheel rotation)
│       ├── robot1/right_wheel   (dynamic — wheel rotation)
│       ├── robot1/caster_wheel  (static)
│       ├── robot1/laser_frame   (static)
│       └── robot1/camera_mast   (static)
│           └── robot1/camera_link (static)
│               └── robot1/camera_link_optical (static)
│
└── robot2/odom  (static TF at spawn position)
    └── robot2/base_link  (dynamic — from diff-drive odometry)
        ├── robot2/left_wheel
        ├── robot2/right_wheel
        ├── robot2/caster_wheel
        ├── robot2/laser_frame
        └── robot2/camera_mast
            └── robot2/camera_link
                └── robot2/camera_link_optical
```

- **Static transforms** (`world → robotX/odom`, `base_link → sensors`) are published by `static_transform_publisher` and `robot_state_publisher`.
- **Dynamic transforms** (`robotX/odom → robotX/base_link`, wheel rotations) come from Gazebo via the bridge.

---

## 8. Key Design Decisions

### Why `OpaqueFunction`?

Standard ROS 2 launch substitutions are evaluated lazily and cannot be used in Python loops or string formatting. `OpaqueFunction` defers execution to a Python callback that receives resolved `LaunchConfiguration` values, enabling dynamic node generation with a `for` loop.

### Why a single bridge node?

Running one `ros_gz_bridge` node with a combined config is more resource-efficient than running N separate bridge nodes. The bridge handles all topic mappings in one process.

### Why prefix-based namespacing instead of ROS namespaces alone?

Gazebo doesn't understand ROS namespaces. By embedding the prefix directly into link names, joint names, and Gazebo topic names at the Xacro level, each robot operates on distinct Gazebo topics (`robot1/cmd_vel`, `robot2/cmd_vel`). The bridge then maps these to ROS namespaced topics.

### Why `world → robotX/odom` static transforms?

Without these, each robot's TF tree would be disconnected. The static transforms anchor every robot in a common `world` frame, enabling multi-robot visualisation in RViz without TF lookup failures.

### Why dynamic bridge YAML generation?

A static YAML file would need to be manually updated whenever the number of robots changes. Generating it programmatically from `num_robots` makes the system fully configurable from a single launch argument.
