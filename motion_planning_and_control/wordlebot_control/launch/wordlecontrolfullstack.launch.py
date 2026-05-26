from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")
    model_path = LaunchConfiguration("model_path")
    hl_startup_delay = LaunchConfiguration("hl_startup_delay")

    wordle_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("wordlebot_control"),
                    "launch",
                    "wordle_bot_mtc.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "onrobot_type": onrobot_type,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "prefix": prefix,
        }.items(),
    )

    hl_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hl_control"),
                    "launch",
                    "hl_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "model_path": model_path,
        }.items(),
    )

    delayed_hl_control_launch = TimerAction(
        period=hl_startup_delay,
        actions=[hl_control_launch],
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
                description="Type of the OnRobot gripper.",
            ),
            DeclareLaunchArgument(
                "safety_limits",
                default_value="true",
                description="Enables the safety limits controller if true.",
            ),
            DeclareLaunchArgument(
                "safety_pos_margin",
                default_value="0.15",
                description="The margin to lower and upper limits in the safety controller.",
            ),
            DeclareLaunchArgument(
                "safety_k_position",
                default_value="20",
                description="k-position factor in the safety controller.",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value='""',
                description="Prefix of the joint names.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description=(
                    "Absolute path to MaskablePPO model without .zip extension. "
                    "Defaults to the installed hl_control model."
                ),
            ),
            DeclareLaunchArgument(
                "hl_startup_delay",
                default_value="0.0",
                description="Seconds to wait after starting Wordle control before HL control.",
            ),
            wordle_control_launch,
            delayed_hl_control_launch,
        ]
    )
