import os
import math
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction callback – has access to resolved LaunchConfigurations."""

    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_practica1 = get_package_share_directory('practica1')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_practica1, 'urdf', 'robotito.urdf.xacro')
    world_file = os.path.join(pkg_robot_bringup, 'world', 'my_world.sdf')

    # ── Gazebo Sim (only once) ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    entities = [gazebo]

    # ── Spawn positions (circle around origin) ────────────────────────────
    radius = 2.0
    spawn_positions = []
    for i in range(num_robots):
        angle = 2.0 * math.pi * i / num_robots
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        spawn_positions.append((x, y))

    # ── Bridge config (built dynamically) ─────────────────────────────────
    bridge_entries = [
        # /clock is global – only one entry
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        },
    ]

    for i in range(num_robots):
        ns = f'robot{i+1}'
        prefix = f'{ns}/'

        # Per-robot bridge topics
        per_robot = [
            {
                'ros_topic_name': f'/{ns}/cmd_vel',
                'gz_topic_name': f'/{prefix}cmd_vel',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
            },
            {
                'ros_topic_name': f'/{ns}/joint_states',
                'gz_topic_name': f'/{prefix}joint_states',
                'ros_type_name': 'sensor_msgs/msg/JointState',
                'gz_type_name': 'gz.msgs.Model',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{ns}/scan',
                'gz_topic_name': f'/{prefix}scan',
                'ros_type_name': 'sensor_msgs/msg/LaserScan',
                'gz_type_name': 'gz.msgs.LaserScan',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{ns}/camera/image_raw',
                'gz_topic_name': f'/{prefix}camera/image_raw',
                'ros_type_name': 'sensor_msgs/msg/Image',
                'gz_type_name': 'gz.msgs.Image',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{ns}/odom',
                'gz_topic_name': f'/{prefix}odom',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': '/tf',
                'gz_topic_name': f'/{prefix}tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        ]
        bridge_entries.extend(per_robot)

    # Write YAML to a temp file so the bridge node can read it
    bridge_yaml_path = os.path.join(
        tempfile.gettempdir(), 'multi_robot_bridge_config.yaml'
    )
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_entries, f, default_flow_style=False)

    # Bridge node (single node, all topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_yaml_path,
            'use_sim_time': True,
        }],
        output='screen',
    )
    entities.append(bridge)

    # ── Per-robot: xacro → RSP → spawn ───────────────────────────────────
    for i in range(num_robots):
        ns = f'robot{i+1}'
        prefix = f'{ns}/'
        x, y = spawn_positions[i]

        # Process xacro with the robot-specific prefix
        doc = xacro.process_file(xacro_file, mappings={'prefix': prefix})
        robot_desc = doc.toxml()

        # Robot State Publisher (namespaced, but TFs go to global /tf)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True},
            ],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ],
        )

        # Spawn entity
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', robot_desc,
                '-name', ns,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.1',
            ],
            output='screen',
        )

        # Static transform: world → <ns>/odom (anchors each robot in the global frame)
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{ns}_world_to_odom',
            arguments=[
                '--x', str(x),
                '--y', str(y),
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', f'{ns}/odom',
            ],
            output='screen',
        )

        entities.extend([rsp, spawn, static_tf])

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='Number of robots to spawn',
        ),
        OpaqueFunction(function=launch_setup),
    ])
