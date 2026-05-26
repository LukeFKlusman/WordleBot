import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.substitutions import FindPackageShare

WS_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
PERCEPTION_SCRIPT   = os.path.join(WS_ROOT, 'perception', 'src', 'realsense_camera_cnn.py')
GAMIFICATION_SCRIPT = os.path.join(WS_ROOT, 'gamification', 'gamification_trial.py')

def generate_launch_description():
    human_detection_arg = DeclareLaunchArgument(
        'human_detection', default_value='false',
        description='Enable MediaPipe human detection (true/false).')
    human_detection = LaunchConfiguration('human_detection')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_launch.py']),
        launch_arguments={'enable_depth': 'true', 'align_depth.enable': 'true'}.items())

    log_start = LogInfo(msg=[
        '\n================================================\n'
        '  SS1 Perception Stack — WordleBot CELK RS2\n'
        '  RealSense driver starting...\n'
        '  Perception node starts in 5 seconds.\n'
        '  Gamification node starts in 10 seconds.\n'
        '  SPACEBAR = toggle scan  |  Q = quit\n'
        '================================================'])

    perception_node = TimerAction(period=5.0, actions=[
        LogInfo(msg='Starting realsense_camera_cnn.py...'),
        ExecuteProcess(
            cmd=[FindExecutable(name='python3'), PERCEPTION_SCRIPT],
            cwd=WS_ROOT,
            additional_env={'SS1_HUMAN_DETECTION': human_detection},
            output='screen', shell=False)])

    gamification_node = TimerAction(period=10.0, actions=[
        LogInfo(msg='Starting gamification_trial.py...'),
        ExecuteProcess(
            cmd=[FindExecutable(name='python3'), GAMIFICATION_SCRIPT],
            cwd=os.path.join(WS_ROOT, 'gamification'),
            output='screen', shell=False)])

    return LaunchDescription([
        human_detection_arg, log_start,
        realsense_launch, perception_node, gamification_node])