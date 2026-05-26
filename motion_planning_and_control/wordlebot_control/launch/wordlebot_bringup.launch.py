from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    driver_startup_delay = LaunchConfiguration("driver_startup_delay")
    rviz_startup_delay = LaunchConfiguration("rviz_startup_delay")

    robot_driver_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_onrobot_control"),
                            "launch",
                            "start_robot.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "onrobot_type": onrobot_type,
                    "use_fake_hardware": use_fake_hardware,
                    "robot_ip": robot_ip,
                    "launch_rviz": "false",
                }.items(),
            )
        ],
    )

    moveit_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_onrobot_moveit_config"),
                            "launch",
                            "ur_onrobot_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "onrobot_type": onrobot_type,
                    "launch_rviz": "false",
                }.items(),
            )
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("wordlebot_control"),
                    "launch",
                    "wordlebot_rviz.launch.py",
                ]
            )
        ),
        condition=IfCondition(launch_rviz),
    )

    delayed_rviz_launch = TimerAction(
        period=rviz_startup_delay,
        actions=[rviz_launch],
    )

    delayed_moveit_launch = TimerAction(
        period=driver_startup_delay,
        actions=[moveit_launch, delayed_rviz_launch],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                description="Type/series of used UR robot.",
            ),
            DeclareLaunchArgument(
                "onrobot_type",
                default_value="rg2",
                description="Type/series of used OnRobot gripper.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Start robot with fake hardware mirroring commands to states.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.0.194",
                description="IP address by which the robot can be reached.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch the WordleBot RViz configuration.",
            ),
            DeclareLaunchArgument(
                "driver_startup_delay",
                default_value="10.0",
                description="Seconds to wait after starting the robot driver before MoveIt.",
            ),
            DeclareLaunchArgument(
                "rviz_startup_delay",
                default_value="3.0",
                description="Seconds to wait after starting MoveIt before RViz.",
            ),
            robot_driver_launch,
            delayed_moveit_launch,
        ]
    )
