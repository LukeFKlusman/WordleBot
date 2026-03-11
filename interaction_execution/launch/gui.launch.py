from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
        Node(
            package="interaction_execution",
            executable="interaction_execution_node",
            output="screen",
        ),
    ])
