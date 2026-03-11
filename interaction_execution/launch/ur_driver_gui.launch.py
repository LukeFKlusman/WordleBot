from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_driver = LaunchConfiguration("launch_driver")

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        condition=IfCondition(launch_driver),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": "false",
        }.items(),
    )

    gui_node = Node(
        package="interaction_execution",
        executable="interaction_execution_node",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", description="UR robot type, e.g. ur3e"),
            DeclareLaunchArgument("robot_ip", description="Robot or Polyscope simulator IP"),
            DeclareLaunchArgument(
                "launch_driver",
                default_value="false",
                description="Launch ur_robot_driver together with the GUI",
            ),
            SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
            ur_driver_launch,
            gui_node,
        ]
    )
