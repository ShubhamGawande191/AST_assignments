from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robile_safety',
            executable='safety_monitoring_smach',
            name='safety_monitoring_smach_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
    ])
