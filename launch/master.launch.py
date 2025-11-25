from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='master_controller',
            name='master_controller',
            output='screen',
        ),
        Node(
            package='robot_controller',
            executable='data_logger',
            name='data_logger',
            output='screen',
        ),
    ])