"""
Launch file for the goal_motion integration tests.

Assumes the UR robot driver and MoveIt stack are already running
(as per the README: ur_control.launch.py + ur_moveit.launch.py).
This launch file starts only the wordleBot_control node so that
the test runner can publish goals and validate motion.
"""

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions


def generate_launch_description():
    control_node = launch_ros.actions.Node(
        package="wordleBot_control",
        executable="wordleBot_control",
        name="wordle_bot_control_node",
        output="screen",
    )

    return launch.LaunchDescription([
        control_node,
        # Signal to launch_testing that the system is ready for tests
        launch_testing.actions.ReadyToTest(),
    ])
