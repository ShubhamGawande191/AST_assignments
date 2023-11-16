from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robile_safety',
            executable='safety_monitoring_bt',
            name='safety_monitoring_bt_node',
            output='screen',
        ),
    ])
