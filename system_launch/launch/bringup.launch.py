from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declarar argumentos para pasar desde la línea de comandos
    return LaunchDescription([
        DeclareLaunchArgument('device_path', default_value='/dev/video4', description='Ruta de la cámara'),
        DeclareLaunchArgument('arduino_path', default_value='/dev/ttyACM0', description='Ruta del Arduino'),

        Node(
            package='px2',
            executable='px2',
            name='px2_node',
            output='screen'
        ),
        Node(
            package='hw_px3',
            executable='hw_px3',
            name='hw_px3_node',
            output='screen',
            parameters=[{'arduino_path': LaunchConfiguration('arduino_path')}]
        ),
        Node(
            package='my_camera_pkg',
            executable='camera_publisher',
            name='my_camera_pkg_node',
            output='screen',
            parameters=[{'device_path': LaunchConfiguration('device_path')}]
        ),
    ])
