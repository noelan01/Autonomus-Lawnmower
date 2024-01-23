#!/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    wheel_sync_node = Node(
        package="hqv_sync_sensors",
        executable="sync_wheels",
        name="sync_wheels",
        remappings=[],
        parameters=[
            {"queue_size": 10},
            {"max_jitter": 0.1},
        ],
    )

    ld.add_action(wheel_sync_node)

    return ld
