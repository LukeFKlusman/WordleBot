# Not the used launch file - see wordle_bot_mtc.launch.py for the MTC version.  This is left here for reference and comparison.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, relative_path):
    pkg_path = get_package_share_directory(package_name)
    with open(os.path.join(pkg_path, relative_path), "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_share = get_package_share_directory("wordlebot_control")
    kinematics_yaml_path = os.path.join(pkg_share, "config", "kinematics.yaml")

    with open(kinematics_yaml_path, "r") as f:
        kinematics_config = yaml.safe_load(f)

    # Extract the robot_description_kinematics params nested under /**:ros__parameters:
    kin_params = (
        kinematics_config.get("/**", {})
        .get("ros__parameters", {})
    )

    # MTC's PipelinePlanner instantiates the planning pipeline in-process and needs
    # these parameters in the node's namespace.  Without them it falls back to CHOMP
    # and segfaults.  MoveGroupInterface is unaffected (it delegates to move_group node).
    ompl_planning = load_yaml(
        "ur_moveit_config", "config/ompl_planning.yaml"
    )
    # ompl_planning.yaml only has planner_configs; MTC PipelinePlanner("ompl") needs
    # ompl.planning_plugin to exist or it falls back to CHOMP and hangs.
    ompl_planning["planning_plugin"] = "ompl_interface/OMPLPlanner"
    ompl_planning["request_adapters"] = (
        "default_planner_request_adapters/AddTimeOptimalParameterization "
        "default_planner_request_adapters/FixWorkspaceBounds "
        "default_planner_request_adapters/FixStartStateBounds "
        "default_planner_request_adapters/FixStartStateCollision "
        "default_planner_request_adapters/FixStartStatePathConstraints"
    )
    joint_limits = load_yaml(
        "ur_moveit_config", "config/joint_limits.yaml"
    )

    wordlebot_control = Node(
        package="wordlebot_control",
        executable="wordlebot_control_node",
        output="screen",
        parameters=[
            kin_params,
            {"ompl": ompl_planning},
            joint_limits,
        ],
    )

    return LaunchDescription([wordlebot_control])
