from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory("wordleBot_control")
    kinematics_yaml_path = os.path.join(pkg_share, "config", "kinematics.yaml")

    with open(kinematics_yaml_path, "r") as f:
        kinematics_config = yaml.safe_load(f)

    # Extract the robot_description_kinematics params nested under /**:ros__parameters:
    kin_params = (
        kinematics_config.get("/**", {})
        .get("ros__parameters", {})
    )

    wordleBot_control = Node(
        package="wordleBot_control",
        executable="wordleBot_control",
        output="screen",
        parameters=[kin_params],
    )

    return LaunchDescription([wordleBot_control])
