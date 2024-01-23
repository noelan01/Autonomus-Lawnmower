from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hqv_proxy',
            executable='hrp',
            name='hrp',
            parameters=[{
                "target": "loop_can"
            }]
        ),
        Node(
            package='hqv_power_shutdown_linux',
            executable='main',
            name='hqv_power_shutdown_linux'
        ),
        Node(
            package='hqv_mower_keep_alive',
            executable='main',
            name='hqv_mower_keep_alive'
        ),
    ])
