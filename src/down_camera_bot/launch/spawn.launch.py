import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('down_camera_bot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'cubo_camara.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'camera.sdf')
    
    # Leer contenido del URDF para robot_state_publisher
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Iniciar el nuevo Gazebo (Harmonic) con un mundo vacío
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        ),
        # Publicar TF del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        # Generar el cubo en Gazebo a 20 metros
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'cubo_camara', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '20.0'],
            output='screen'
        ),
        # Puente traductor ROS 2 <-> Gazebo para la cámara
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen'
        )
    ])
