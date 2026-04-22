from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

import os
from pathlib import Path


def find_package_share(package_name):
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        return None


def generate_launch_description():
    launch_gui = LaunchConfiguration("launch_gui")
    launch_motion = LaunchConfiguration("launch_motion")
    launch_perception = LaunchConfiguration("launch_perception")
    launch_gamification = LaunchConfiguration("launch_gamification")
    launch_voice_control = LaunchConfiguration("launch_voice_control")

    wordle_bot_share = find_package_share("wordleBot_control")
    moveit_config_share = find_package_share("ur_onrobot_moveit_config")
    wordle_bot_launch = (
        wordle_bot_share / "launch" / "wordle_bot.launch.py"
        if wordle_bot_share is not None
        else None
    )

    workspace_root = Path(get_package_share_directory("interaction_execution")).resolve().parents[4]
    repo_root = workspace_root / "src" / "RS2"
    perception_script = repo_root / "perception" / "src" / "realsense_camera_cnn.py"
    gamification_script = repo_root / "gamification" / "gamificiation_node.py"
    voice_control_script = repo_root / "voice_control" / "main.py"

    launch_actions = [
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
        DeclareLaunchArgument(
            "launch_perception",
            default_value="true",
            description="Launch the perception script alongside the GUI.",
        ),
        DeclareLaunchArgument(
            "launch_gamification",
            default_value="true",
            description="Launch the gamification node alongside the GUI.",
        ),
        DeclareLaunchArgument(
            "launch_voice_control",
            default_value="false",
            description="Launch the voice_control CLI script. Disabled by default because it is interactive.",
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
        ExecuteProcess(
            cmd=["python3", str(perception_script)],
            output="screen",
            condition=IfCondition(launch_perception),
        ),
        ExecuteProcess(
            cmd=["python3", str(gamification_script)],
            output="screen",
            condition=IfCondition(launch_gamification),
        ),
        ExecuteProcess(
            cmd=["python3", str(voice_control_script)],
            output="screen",
            condition=IfCondition(launch_voice_control),
        ),
    ]

    if wordle_bot_launch is not None and moveit_config_share is not None:
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(wordle_bot_launch)),
                condition=IfCondition(launch_motion),
            )
        )
    else:
        missing_packages = []
        if wordle_bot_share is None:
            missing_packages.append("wordleBot_control")
        if moveit_config_share is None:
            missing_packages.append("ur_onrobot_moveit_config")

        launch_actions.append(
            LogInfo(
                condition=IfCondition(launch_motion),
                msg=(
                    "Skipping motion launch because required package(s) are missing: "
                    + ", ".join(missing_packages)
                ),
            )
        )

    return LaunchDescription(launch_actions)
