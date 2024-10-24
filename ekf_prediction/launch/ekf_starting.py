from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_prediction',
            executable='ekf_prediction',
            name='ekf_node',
            output='screen'
        ),
        Node(
            package='posi_read',
            executable='posi_read',
            name='posi_read',
            output='screen'
        )
    ])
