#!/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    odom_imu_sync_node = Node(
        package="hqv_sync_sensors",
        executable="sync_odom_imu",
        name="sync_odom_imu",
        remappings=[],
        parameters=[
            {"queue_size": 10},
            {"max_jitter": 0.1},
        ],
    )

    ld.add_action(odom_imu_sync_node)

    return ld
