"""
Launch file for Test Case 2 — Advanced Motion Control.
Validates: HD (High Distinction) criteria.

Assumptions:
  The following must already be running before invoking this launch file:
    - UR robot driver:  ros2 launch ur_robot_driver ur_control.launch.py ...
    - MoveIt stack:     ros2 launch ur_moveit_config ur_moveit.launch.py ...

Usage:
  ros2 launch wordlebot_control tc2_advanced_motion_control.launch.py
  # then in a separate terminal:
  colcon test --packages-select wordlebot_control --pytest-args -k tc2
"""

import os

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("wordlebot_control")
    kinematics_yaml_path = os.path.join(pkg_share, "config", "kinematics.yaml")

    with open(kinematics_yaml_path, "r") as f:
        kinematics_config = yaml.safe_load(f)

    kin_params = (
        kinematics_config.get("/**", {})
        .get("ros__parameters", {})
    )

    # WordleBot control node
    control_node = launch_ros.actions.Node(
        package="wordlebot_control",
        executable="wordlebot_control",
        name="wordle_bot_control_node",
        output="screen",
        parameters=[kin_params],
    )

    # TODO (TC2.1): add goal ordering / design matrix node here if it runs as a separate node
    # design_matrix_node = launch_ros.actions.Node(
    #     package="<design_matrix_package>",
    #     executable="<design_matrix_executable>",
    #     name="design_matrix_node",
    #     output="screen",
    # )

    # TODO (TC2.3): add gripper driver / bridge node here once interface is defined
    # gripper_node = launch_ros.actions.Node(
    #     package="<gripper_package>",
    #     executable="<gripper_executable>",
    #     name="gripper_node",
    #     output="screen",
    # )

    return launch.LaunchDescription([
        control_node,
        # TODO: uncomment when design_matrix_node and gripper_node are implemented
        # design_matrix_node,
        # gripper_node,
        launch_testing.actions.ReadyToTest(),
    ])
