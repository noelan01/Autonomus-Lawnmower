from launch import LaunchDescription, actions
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hqv_proxy',
            executable='p2z',
            name='p2z'
        ),
        Node(
            package='hqv_power_shutdown_linux',
            executable='main',
            name='hqv_power_shutdown_linux',
            parameters=[{"mower_app_republish_rate": '5.0'}]
        ),
        actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])
