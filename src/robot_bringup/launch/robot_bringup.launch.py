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


def preparar_lanzamiento(context, *args, **kwargs):

    cantidad_robots = int(LaunchConfiguration('num_robots').perform(context))

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_practica1 = get_package_share_directory('practica1')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    archivo_xacro = os.path.join(pkg_practica1, 'urdf', 'robotito.urdf.xacro')
    archivo_mundo = os.path.join(pkg_robot_bringup, 'world', 'mundo_objetos.sdf')

    # Lanzamiento estandar: Gazebo con cliente GUI siempre activo.
    argumentos_gz = f'-r {archivo_mundo}'

    # para lanzar gazebo
    lanzador_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': argumentos_gz}.items(),
    )

    entidades = [lanzador_gazebo]

    # Posiciones de spawn en circulo, va formando un circulo con radio fijo segun la cantidad de robots
    radio = 2.0
    posiciones_spawn = []
    for i in range(cantidad_robots):
        angulo = 2.0 * math.pi * i / cantidad_robots
        x = radio * math.cos(angulo)
        y = radio * math.sin(angulo)
        posiciones_spawn.append((x, y))

    #Configuracion del bridge
    entradas_bridge = [
        # /clock -> global para sincronizar tiempo entre Gazebo y ROS
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        },
    ]

    for i in range(cantidad_robots):
        ns = f'robot{i+1}'
        prefijo = f'{ns}/'

        # Topics bridge por robot
        por_robot = [
            {
                'ros_topic_name': f'/{ns}/cmd_vel',
                'gz_topic_name': f'/{prefijo}cmd_vel',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
            },
            {
                'ros_topic_name': f'/{ns}/joint_states',
                'gz_topic_name': f'/{prefijo}joint_states',
                'ros_type_name': 'sensor_msgs/msg/JointState',
                'gz_type_name': 'gz.msgs.Model',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{ns}/scan',
                'gz_topic_name': f'/{prefijo}scan',
                'ros_type_name': 'sensor_msgs/msg/LaserScan',
                'gz_type_name': 'gz.msgs.LaserScan',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{ns}/camera/image_raw',
                'gz_topic_name': f'/{prefijo}camera/image_raw',
                'ros_type_name': 'sensor_msgs/msg/Image',
                'gz_type_name': 'gz.msgs.Image',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{ns}/odom',
                'gz_topic_name': f'/{prefijo}odom',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': '/tf',
                'gz_topic_name': f'/{prefijo}tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        ]
        entradas_bridge.extend(por_robot)

    # Guardamos YAML temporal para que lo lea el nodo bridge
    ruta_yaml_bridge = os.path.join(
        tempfile.gettempdir(), 'multi_robot_bridge_config.yaml'
    )
    with open(ruta_yaml_bridge, 'w') as f:
        yaml.dump(entradas_bridge, f, default_flow_style=False)

    # Nodo bridge
    nodo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ruta_yaml_bridge,
            'use_sim_time': True,
        }],
        output='screen',
    )
    entidades.append(nodo_bridge)

    #Por robot: xacro -> RSP -> spawn
    for i in range(cantidad_robots):
        ns = f'robot{i+1}'
        prefijo = f'{ns}/'
        x, y = posiciones_spawn[i]

        # se procesa xacro con prefijo especifico del robot
        doc_xacro = xacro.process_file(archivo_xacro, mappings={'prefix': prefijo})
        descripcion_robot = doc_xacro.toxml()

        # Robot State Publisher con namespace
        publicador_estado_robot = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[
                {'robot_description': descripcion_robot},
                {'use_sim_time': True},
            ],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ],
        )

        # Spawn de la entidad
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', descripcion_robot,
                '-name', ns,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.1',
            ],
            output='screen',
        )

        # tf estatica: world -> <ns>/odom para anclar cada robot
        tf_estatico = Node(
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

        entidades.extend([publicador_estado_robot, spawn_robot, tf_estatico])

    return entidades


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='2', #por defecto 2 robots
            description='Cantidad de robots a spawnear',
        ),
        OpaqueFunction(function=preparar_lanzamiento), #usamos OpaqueFunction para tener acceso a LaunchConfiguration durante la preparacin del lanzamiento
    ])
