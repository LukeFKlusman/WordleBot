"""
hl_control.launch.py

Launch the high-level Wordle-bot control node.

Arguments
---------
use_test_publisher : bool   (default false)
    When true, also spawn the TC2.1 test environment publisher.

word : str   (default CRANE)
    Target word passed to the test publisher. Ignored when use_test_publisher=false.

config_path : str
    Path to the board YAML loaded by the test publisher.
    Defaults to tc2_1_board.yaml in the hl_control config directory.

model_path : str
    Absolute path (without .zip extension) to the MaskablePPO model.
    Defaults to the models/wordle_ppo_latest checkpoint in rl_task_optimiser.

rl_task_optimiser_path : str
    Absolute path to the rl_task_optimiser directory so its modules can be imported.
    Defaults to /home/connorlindsell/git/AiRobotics/rl_task_optimiser.

Usage examples
--------------
# TC2.1 full pipeline:
ros2 launch hl_control hl_control.launch.py use_test_publisher:=true word:=CRANE

# Standalone HL node (perception / solver provided externally):
ros2 launch hl_control hl_control.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_RL_DEFAULT = '/home/connorlindsell/git/AiRobotics/rl_task_optimiser'


def launch_setup(context, *args, **kwargs):
    rl_path = LaunchConfiguration('rl_task_optimiser_path').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    config_path = LaunchConfiguration('config_path').perform(context)
    word = LaunchConfiguration('word').perform(context)
    use_test_pub = LaunchConfiguration('use_test_publisher').perform(context).lower()

    if not model_path:
        model_path = os.path.join(rl_path, 'models', 'wordle_ppo_latest')

    if not config_path:
        config_path = os.path.join(
            FindPackageShare('hl_control').perform(context),
            'config',
            'tc2_1_board.yaml',
        )

    # PYTHONPATH must include both the rl_task_optimiser root (for `from test import ...`)
    # and its parent (for `from training_env.wordle_env import ...`).
    extra_python_path = os.pathsep.join([rl_path, os.path.dirname(rl_path)])
    existing_python_path = os.environ.get('PYTHONPATH', '')
    full_python_path = (
        extra_python_path + os.pathsep + existing_python_path
        if existing_python_path
        else extra_python_path
    )

    shared_env = {'PYTHONPATH': full_python_path}

    hl_node = Node(
        package='hl_control',
        executable='hl_control_node.py',
        name='hl_control_node',
        output='screen',
        parameters=[{'model_path': model_path}],
        additional_env=shared_env,
    )

    nodes = [hl_node]

    if use_test_pub in ('true', '1', 'yes'):
        test_node = Node(
            package='hl_control',
            executable='test_env_publisher.py',
            name='test_env_publisher',
            output='screen',
            parameters=[{
                'config_path':     config_path,
                'word':            word,
                'publish_delay':   2.0,
            }],
            additional_env=shared_env,
        )
        nodes.append(test_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_test_publisher',
            default_value='false',
            description='Spawn the TC2.1 test environment publisher node.',
        ),
        DeclareLaunchArgument(
            'word',
            default_value='CRANE',
            description='Target word for the test publisher.',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description='Path to board YAML (default: tc2_1_board.yaml in hl_control/config).',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Absolute path to MaskablePPO model (without .zip). '
                        'Defaults to rl_task_optimiser/models/wordle_ppo_latest.',
        ),
        DeclareLaunchArgument(
            'rl_task_optimiser_path',
            default_value=_RL_DEFAULT,
            description='Path to the rl_task_optimiser package root directory.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
