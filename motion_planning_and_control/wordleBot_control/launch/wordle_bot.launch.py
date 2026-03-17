from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # MoveIt is launched separately for now

    wordleBot_control = Node(
        package="wordleBot_control",
        executable="wordleBot_control",
        output="screen",
    )

    return LaunchDescription([wordleBot_control])


