from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    keyboard_remote_drive = Node(
        package="hqv_keyboard_remote_drive",
        executable="keyboard_remote_drive",
        name="keyboard_remote_drive",
        remappings=[],
        parameters=[],
    )

    ld.add_action(keyboard_remote_drive)

    return ld
