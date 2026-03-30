"""
Launch file for Test Case 3 — Advanced Collision Avoidance.
Validates: Perfect criteria.

Assumptions:
  The following must already be running before invoking this launch file:
    - UR robot driver:  ros2 launch ur_robot_driver ur_control.launch.py ...
    - MoveIt stack:     ros2 launch ur_moveit_config ur_moveit.launch.py ...

Usage:
  ros2 launch wordleBot_control tc3_advanced_collision_avoidance.launch.py
  # then in a separate terminal:
  colcon test --packages-select wordleBot_control --pytest-args -k tc3
"""

import os

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("wordleBot_control")
    kinematics_yaml_path = os.path.join(pkg_share, "config", "kinematics.yaml")

    with open(kinematics_yaml_path, "r") as f:
        kinematics_config = yaml.safe_load(f)

    kin_params = (
        kinematics_config.get("/**", {})
        .get("ros__parameters", {})
    )

    # WordleBot control node
    control_node = launch_ros.actions.Node(
        package="wordleBot_control",
        executable="wordleBot_control",
        name="wordle_bot_control_node",
        output="screen",
        parameters=[kin_params],
    )

    # TODO (TC3.1/TC3.2): add any active collision monitor node here if it runs as a separate node
    # (e.g., a node that watches the planning scene and triggers replanning when a new obstacle appears)
    # collision_monitor_node = launch_ros.actions.Node(
    #     package="<collision_monitor_package>",
    #     executable="<collision_monitor_executable>",
    #     name="collision_monitor_node",
    #     output="screen",
    # )

    # TODO (TC3.3): add gripper driver / bridge node here once interface is defined
    # gripper_node = launch_ros.actions.Node(
    #     package="<gripper_package>",
    #     executable="<gripper_executable>",
    #     name="gripper_node",
    #     output="screen",
    # )

    return launch.LaunchDescription([
        control_node,
        # TODO: uncomment when collision_monitor_node and gripper_node are implemented
        # collision_monitor_node,
        # gripper_node,
        launch_testing.actions.ReadyToTest(),
    ])
