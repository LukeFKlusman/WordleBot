from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    launch_gui = LaunchConfiguration("launch_gui")
    launch_motion = LaunchConfiguration("launch_motion")

    wordle_bot_launch = os.path.join(
        get_package_share_directory("wordleBot_control"),
        "launch",
        "wordle_bot.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_gui",
            default_value="true",
            description="Launch the interaction_execution GUI and mission coordinator.",
        ),
        DeclareLaunchArgument(
            "launch_motion",
            default_value="true",
            description="Launch the wordleBot motion package alongside the GUI.",
        ),
        SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
        Node(
            package="interaction_execution",
            executable="interaction_execution_node",
            output="screen",
            condition=IfCondition(launch_gui),
        ),
        Node(
            package="interaction_execution",
            executable="mission_coordinator_node",
            output="screen",
            condition=IfCondition(launch_gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wordle_bot_launch),
            condition=IfCondition(launch_motion),
        ),
    ])