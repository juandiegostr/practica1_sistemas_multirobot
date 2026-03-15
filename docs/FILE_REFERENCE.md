# File-by-File Reference

> Detailed description of every file in the `src/` directory of the multi-robot workspace.

---

## Table of Contents

- [Package `practica1`](#package-practica1)
  - [package.xml](#practica1packagexml)
  - [CMakeLists.txt](#practica1cmakeliststxt)
  - [urdf/robotito.urdf.xacro](#urdfrobotitourdxacro)
  - [urdf/inertial_macros.xacro](#urdfinertial_macrosxacro)
  - [urdf/gazebo_control.xacro](#urdfgazebo_controlxacro)
  - [urdf/lidar.xacro](#urdflidarxacro)
  - [urdf/camera.xacro](#urdfcameraxacro)
  - [urdf/robotito.urdf](#urdfrobitourdf)
  - [launch/sencillo_launch.launch](#launchsencillo_launchlaunch)
- [Package `robot_bringup`](#package-robot_bringup)
  - [package.xml](#robot_bringuppackagexml)
  - [CMakeLists.txt](#robot_bringupcmakeliststxt)
  - [launch/robot_bringup.launch.py](#launchrobot_bringuplaunchy)
  - [config/bridge_config.yaml](#configbridge_configyaml)
  - [world/my_world.sdf](#worldmy_worldsdf)

---

## Package `practica1`

### `practica1/package.xml`

Standard ROS 2 package manifest (format 3).

| Field | Value |
|-------|-------|
| Name | `practica1` |
| Build system | `ament_cmake` |
| Dependencies | `rclcpp`, `rclpy` |
| Test dependencies | `ament_lint_auto`, `ament_lint_common` |

The runtime dependencies (`rclcpp`, `rclpy`) are declared but not actually compiled against — the package installs only static assets.

---

### `practica1/CMakeLists.txt`

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY urdf  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
```

**What it does:**
1. Finds build dependencies (though no targets use them).
2. Installs the `urdf/` directory to `share/practica1/urdf/` — this is how `get_package_share_directory('practica1')` can locate the xacro files at runtime.
3. Installs the `launch/` directory to `share/practica1/launch/`.

---

### `urdf/robotito.urdf.xacro`

**Role:** Main robot description entry point. Assembles the full robot from sub-files.

**Key elements:**

#### Prefix mechanism
```xml
<xacro:arg name="prefix" default="" />
<xacro:property name="ns_prefix" value="$(arg prefix)" />
```
- `prefix` is passed at process time (e.g., `robot1/`).
- Every link/joint uses `${ns_prefix}` so names become `robot1/base_link`, `robot1/left_wheel_joint`, etc.
- Default is empty string → works as a normal single-robot URDF.

#### Includes
```xml
<xacro:include filename="inertial_macros.xacro" />
<xacro:include filename="gazebo_control.xacro" />
<xacro:include filename="lidar.xacro" />
<xacro:include filename="camera.xacro" />
```

#### Properties (dimensions)
```xml
<xacro:property name="chassis_length" value="0.4" />    <!-- 40 cm -->
<xacro:property name="chassis_width" value="0.3" />     <!-- 30 cm -->
<xacro:property name="chassis_height" value="0.1" />    <!-- 10 cm -->
<xacro:property name="wheel_radius" value="0.1" />      <!-- 10 cm -->
<xacro:property name="wheel_thickness" value="0.04" />   <!-- 4 cm -->
<xacro:property name="wheel_offset_x" value="0.1" />
<xacro:property name="wheel_offset_y" value="0.1705" />
<xacro:property name="caster_wheel_radius" value="0.025" />
```

#### base_link
- Green box (visual + collision + inertia).
- Origin at centre of chassis.

#### Wheel macro
```xml
<xacro:macro name="wheel" params="prefix offset_y">
```
A reusable macro that creates a wheel link + continuous joint. Called twice:
```xml
<xacro:wheel prefix="left" offset_y="${wheel_offset_y}"/>
<xacro:wheel prefix="right" offset_y="${-wheel_offset_y}"/>
```
- Creates `${ns_prefix}left_wheel` and `${ns_prefix}right_wheel`.
- Joint axis is `y` (rolls forward/backward).
- Positioned at `x = -0.1` (slightly behind centre for stability).

#### Caster wheel
- Small white sphere (r = 2.5 cm), fixed to front-right of chassis.
- Position: `x = +0.15, z = -0.075` (below chassis).

---

### `urdf/inertial_macros.xacro`

Three xacro macros for computing `<inertial>` blocks:

#### `inertial_sphere(mass, radius)`
```xml
<inertia ixx="${(2/5) * mass * (radius*radius)}" ... />
```

#### `inertial_box(mass, x, y, z)`
```xml
<inertia ixx="${(1/12) * mass * (y*y + z*z)}"
         iyy="${(1/12) * mass * (x*x + z*z)}"
         izz="${(1/12) * mass * (x*x + y*y)}" />
```

#### `inertial_cylinder(mass, length, radius)`
```xml
<inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}"
         izz="${(1/2) * mass * (radius*radius)}" />
```

All off-diagonal terms (`ixy`, `ixz`, `iyz`) are zero (principal axes aligned with geometry axes).

---

### `urdf/gazebo_control.xacro`

Attaches Gazebo Sim plugins. Uses `$(arg prefix)` to prepend namespace to all joint and topic references.

#### 1. Caster friction
```xml
<gazebo reference="$(arg prefix)caster_wheel">
    <mu1 value="0.1" /><mu2 value="0.1" />
</gazebo>
```
Low friction coefficients allow the caster to slide without resisting chassis rotation.

#### 2. Differential Drive (`gz-sim-diff-drive-system`)

Subscribes to `{prefix}cmd_vel` (Twist) and drives the wheel joints.

| Output | Topic |
|--------|-------|
| Odometry | `{prefix}odom` |
| TF (odom→base_link) | `{prefix}tf` |

Configuration:
- Wheel separation: 0.35 m (distance between left and right wheel centres)
- Wheel radius: 0.1 m
- Odom publish rate: 1 Hz

#### 3. Joint State Publisher (`gz-sim-joint-state-publisher-system`)

Publishes wheel positions on `{prefix}joint_states`. Lists both wheel joints explicitly.

---

### `urdf/lidar.xacro`

#### Physical structure
- `laser_joint` (fixed): attaches `laser_frame` to `base_link` at `(0.1, 0, 0.4)`.
- `laser_frame`: black cylinder (r=5 cm, h=4 cm) with proper inertia.

#### Gazebo sensor
```xml
<sensor name="lidar" type="gpu_lidar">
```
- **360 horizontal samples** covering full circle (−π to +π).
- Range: 0.3 m – 12 m.
- Gaussian noise: mean=0, stddev=0.01.
- Update rate: 10 Hz.
- Publishes on Gazebo topic `{prefix}scan`.
- Frame: `{prefix}laser_frame`.

---

### `urdf/camera.xacro`

#### Physical structure
```
base_link
└── camera_mast_joint (fixed, xyz="0.10 0 0.20")
    └── camera_mast (white cylinder r=2cm, h=30cm)
        └── camera_joint (fixed, xyz="0.02 0 0.15")
            └── camera_link (blue box 5×5×5 cm)
                └── camera_optical_joint (fixed, rpy="-π/2 0 -π/2")
                    └── camera_link_optical (empty link)
```

The optical joint rotation converts:
- Robot frame: x-forward, y-left, z-up
- Optical frame: z-forward, x-right, y-down

This is the standard ROS convention for camera frames, required by `image_proc`, OpenCV, etc.

#### Gazebo sensor
```xml
<sensor name="camera" type="camera">
```
- Resolution: 800×800 (R8G8B8).
- HFOV: ~80° (1.3963 rad).
- Clip: 0.02 – 300 m.
- 30 fps.
- Topic: `{prefix}camera/image_raw`.
- Frame: `{prefix}camera_link_optical`.

---

### `urdf/robotito.urdf`

A **plain URDF** (no xacro) for the basic robot without sensors or plugins.

Contains only:
- `base_link` (green box)
- `left_wheel` + `left_wheel_joint`
- `right_wheel` + `right_wheel_joint`
- `caster_wheel` + `caster_joint`

No `<gazebo>` elements. No camera or lidar. No namespace prefixing. Used by `sencillo_launch.launch` for basic RViz viewing.

---

### `launch/sencillo_launch.launch`

**Format:** XML launch file (ROS 2 XML launch format).

**What it launches:**

| Node | Package | Purpose |
|------|---------|---------|
| `robot_state_publisher` | `robot_state_publisher` | Reads URDF from `robotito.urdf`, publishes `/robot_description` and `/tf_static` |
| `joint_state_publisher` | `joint_state_publisher` | GUI-less joint state publisher (publishes zeroed joint states) |
| `rviz2` | `rviz2` | Visualisation tool |

**How it loads the URDF:**
```xml
<let name="urdf_path" value="$(find-pkg-share practica1)/urdf/robotito.urdf" />
<param name="robot_description" value="$(command 'cat $(var urdf_path)')" />
```
Uses `$(command 'cat ...')` to read the file contents into the `robot_description` parameter.

---

## Package `robot_bringup`

### `robot_bringup/package.xml`

| Field | Value |
|-------|-------|
| Name | `robot_bringup` |
| Build system | `ament_cmake` |
| Dependencies | `rclcpp`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces` |
| Exec dependencies | `robot_state_publisher`, `practica1` |

The dependency on `practica1` ensures that the URDF files are available at runtime.

---

### `robot_bringup/CMakeLists.txt`

```cmake
install(DIRECTORY launch world config DESTINATION share/${PROJECT_NAME})
```

Installs three directories to the share space:
- `launch/` → `share/robot_bringup/launch/`
- `world/` → `share/robot_bringup/world/`
- `config/` → `share/robot_bringup/config/`

No compiled targets.

---

### `launch/robot_bringup.launch.py`

*(See the detailed walkthrough in [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md), section 4.2.1)*

**Summary of the `launch_setup()` function:**

```
launch_setup(context):
│
├── 1. Resolve num_robots from LaunchConfiguration
├── 2. Locate package share directories
├── 3. Launch Gazebo Sim (1 instance)
├── 4. Compute spawn positions on a circle (radius=2m)
├── 5. Build bridge YAML dynamically:
│   ├── /clock (global)
│   └── For each robot: cmd_vel, joint_states, scan, camera, odom, tf
├── 6. Write YAML to /tmp/multi_robot_bridge_config.yaml
├── 7. Create single ros_gz_bridge node
└── 8. For each robot:
    ├── Process xacro with prefix=robotX/
    ├── Create robot_state_publisher node (namespaced)
    ├── Create spawn node (ros_gz_sim create)
    └── Create static_transform_publisher (world → robotX/odom)
```

**Key imports and their purpose:**

| Import | Used for |
|--------|----------|
| `math` | Computing circular spawn positions (cos, sin) |
| `yaml` | Serialising bridge config to YAML |
| `tempfile` | Getting temp directory for bridge config |
| `xacro` | Processing `.xacro` files with prefix mappings |
| `OpaqueFunction` | Deferring node creation until arguments are resolved |

---

### `config/bridge_config.yaml`

Static bridge configuration for a **single un-namespaced robot**. Maps 6 topics:

| # | ROS 2 Topic | Direction | Message Type |
|---|-------------|-----------|-------------|
| 1 | `/clock` | GZ → ROS | `rosgraph_msgs/Clock` |
| 2 | `/joint_states` | GZ → ROS | `sensor_msgs/JointState` |
| 3 | `/tf` | GZ → ROS | `tf2_msgs/TFMessage` |
| 4 | `/cmd_vel` | ROS → GZ | `geometry_msgs/Twist` |
| 5 | `/scan` | GZ → ROS | `sensor_msgs/LaserScan` |
| 6 | `/camera/image_raw` | GZ → ROS | `sensor_msgs/Image` |

> **Note:** This file is NOT used by the multi-robot launch. It is kept as a reference for single-robot setups or manual bridge testing.

---

### `world/my_world.sdf`

**Format:** SDF 1.6

**World name:** `default`

**Physics:**
- Step size: 1 ms
- Real-time factor: 1.0 (attempts real-time simulation)

**Plugins loaded:**

| Plugin | System | Purpose |
|--------|--------|---------|
| `gz-sim-physics-system` | `Physics` | Rigid body dynamics, collision detection |
| `gz-sim-user-commands-system` | `UserCommands` | Enables runtime model spawn/delete via service calls |
| `gz-sim-scene-broadcaster-system` | `SceneBroadcaster` | Publishes scene/model state for GUIs and other tools |
| `gz-sim-sensors-system` | `Sensors` | Processes sensor plugins (camera, lidar) using `ogre2` renderer |

**Scene elements:**
1. **Sun** — directional light at `(0, 0, 10)` with shadows, pointing down at an angle.
2. **Ground plane** — 100×100 m static model, grey material, with collision.

The world is kept minimal — robots are added dynamically at launch time.
