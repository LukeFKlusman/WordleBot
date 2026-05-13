"""
hl_control.launch.py

Launch the high-level Wordle-bot control node.

The node runs as a passive background process and only acts once it receives
both a word request (/hl_control/word_request) and a board state
(/perception/gameboard_state). These are provided by the perception stack and
solver in the full system, or by test/test_sim.py for isolated testing.

Arguments
---------
model_path : str  (optional)
    Absolute path to the MaskablePPO model checkpoint, without the .zip
    extension. Defaults to models/wordle_ppo_latest inside the installed
    package share directory.

Usage examples
--------------
# Standard launch (full system — perception and solver provided externally):
ros2 launch hl_control hl_control.launch.py

# Override model checkpoint:
ros2 launch hl_control hl_control.launch.py model_path:=/path/to/model

# Isolated TC2.1 test (run test_sim.py in a second terminal):
ros2 launch hl_control hl_control.launch.py
# then: python3 test/test_sim.py --ros-args -p config_path:=config/tc2_1_board.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration('model_path').perform(context)

    if not model_path:
        model_path = os.path.join(
            FindPackageShare('hl_control').perform(context),
            '..', '..', 'lib', 'hl_control', 'models', 'wordle_ppo_latest',
        )
        model_path = os.path.normpath(model_path)

    hl_node = Node(
        package='hl_control',
        executable='hl_control_node.py',
        name='hl_control_node',
        output='screen',
        parameters=[{'model_path': model_path}],
    )

    return [hl_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Absolute path to MaskablePPO model (without .zip extension). '
                        'Defaults to models/wordle_ppo_latest in the installed package.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
