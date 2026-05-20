from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    ur_moveit_config_dir = Path(get_package_share_directory("ur_moveit_config"))
    ur_description_dir = Path(get_package_share_directory("ur_description"))

    urdf_file = ur_description_dir / "urdf" / "ur.urdf.xacro"

    robot_description = Command([
        "xacro ",
        str(urdf_file),
        " ur_type:=ur3e",
    ])

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description},
                {"frame_prefix": ""},
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(nodes)
