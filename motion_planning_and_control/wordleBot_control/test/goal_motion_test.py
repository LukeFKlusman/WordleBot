"""
Integration tests for wordleBot_control goal motion.

These tests validate that the robot's end-effector reaches the commanded
goal pose within an acceptable tolerance. Goals are published to the
/wordle_bot/goal_pose topic; completion is signalled by the control node
on /wordle_bot/motion_complete. The end-effector position is read from
/tf (UR driver FK chain) — not from the control logic itself.

Prerequisites (must be running before colcon test):
  - UR robot driver:  ros2 launch ur_robot_driver ur_control.launch.py ...
  - MoveIt stack:     ros2 launch ur_moveit_config ur_moveit.launch.py ...

Run with:
  colcon test --packages-select wordleBot_control
  colcon test-result --verbose
"""

import math
import unittest

import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException


# ---------------------------------------------------------------------------
# Tolerances
# ---------------------------------------------------------------------------
POSITION_TOLERANCE_M = 0.005        # 5 mm
ORIENTATION_TOLERANCE_DEG = 5.0     # degrees
MOTION_TIMEOUT_S = 60.0             # max wait per motion


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_pose_stamped(x: float, y: float, z: float,
                       roll: float = math.pi, pitch: float = 0.0,
                       yaw: float = 0.0,
                       frame_id: str = "ur_base_link") -> PoseStamped:
    """Build a PoseStamped from (x, y, z, roll, pitch, yaw) in the given frame."""
    import tf2_ros  # noqa: F811 — local import avoids circular at module level

    # Convert RPY to quaternion using the same convention as buildPose()
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
    msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
    msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
    msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
    return msg


def _quat_angle_diff_deg(q1, q2) -> float:
    """Return the angular difference between two quaternions in degrees."""
    dot = abs(q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z)
    dot = min(dot, 1.0)  # clamp numerical noise
    return math.degrees(2.0 * math.acos(dot))


# ---------------------------------------------------------------------------
# Test fixture
# ---------------------------------------------------------------------------

@pytest.mark.launch(fixture=launch_testing.fixtures.launch_context_with_nodes)
class TestGoalMotion(unittest.TestCase):
    """Integration tests for wordleBot_control goal reaching."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("goal_motion_test_node")

        cls.goal_pub = cls.node.create_publisher(PoseStamped, "/wordle_bot/goal_pose", 10)

        cls.motion_complete = False
        cls.node.create_subscription(
            Bool,
            "/wordle_bot/motion_complete",
            cls._on_motion_complete,
            10,
        )

        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_motion_complete(cls, msg: Bool):
        if msg.data:
            cls.motion_complete = True

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _send_goal_and_wait(self, pose: PoseStamped) -> None:
        """Publish a goal and block until /wordle_bot/motion_complete fires."""
        import time

        TestGoalMotion.motion_complete = False

        # Brief wait for subscriber handshake
        time.sleep(0.5)

        pose.header.stamp = self.node.get_clock().now().to_msg()
        self.goal_pub.publish(pose)
        self.node.get_logger().info(
            f"Published goal: ({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
        )

        deadline = time.time() + MOTION_TIMEOUT_S
        while not TestGoalMotion.motion_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"Motion did not complete within {MOTION_TIMEOUT_S}s "
                    f"for goal ({pose.pose.position.x:.3f}, "
                    f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
                )

    def _get_ee_transform(self):
        """Look up the current tool0 → ur_base_link transform from /tf."""
        import time

        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    "ur_base_link", "tool0",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
            except TransformException:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        self.fail("Could not look up tool0 → ur_base_link transform within 5 s")

    def _assert_pose_reached(self, goal: PoseStamped) -> None:
        """Validate the EE transform matches the goal within defined tolerances."""
        tf = self._get_ee_transform()
        t = tf.transform.translation
        q_actual = tf.transform.rotation
        q_goal = goal.pose.orientation

        pos_err = math.sqrt(
            (t.x - goal.pose.position.x) ** 2 +
            (t.y - goal.pose.position.y) ** 2 +
            (t.z - goal.pose.position.z) ** 2
        )
        orient_err = _quat_angle_diff_deg(q_actual, q_goal)

        self.node.get_logger().info(
            f"Position error: {pos_err * 1000:.1f} mm | "
            f"Orientation error: {orient_err:.2f} deg"
        )

        self.assertLessEqual(
            pos_err, POSITION_TOLERANCE_M,
            f"Position error {pos_err * 1000:.1f} mm exceeds "
            f"{POSITION_TOLERANCE_M * 1000:.0f} mm tolerance"
        )
        self.assertLessEqual(
            orient_err, ORIENTATION_TOLERANCE_DEG,
            f"Orientation error {orient_err:.2f}° exceeds "
            f"{ORIENTATION_TOLERANCE_DEG:.0f}° tolerance"
        )

    # -----------------------------------------------------------------------
    # TEST 1 — Single goal
    # -----------------------------------------------------------------------

    def test_1_single_goal(self):
        """Robot must reach a single pre-determined goal pose."""
        goal = _make_pose_stamped(0.3, 0.25, 0.25, roll=math.pi)

        self._send_goal_and_wait(goal)
        self._assert_pose_reached(goal)

    # -----------------------------------------------------------------------
    # TEST 2 — Three-goal sequence
    # -----------------------------------------------------------------------

    def test_2_three_goal_sequence(self):
        """Robot must reach three sequential pre-determined goal poses."""
        goals = [
            _make_pose_stamped(-0.2, 0.3,  0.15, roll=math.pi),
            _make_pose_stamped( 0.2, 0.25, 0.20, roll=math.pi),
            _make_pose_stamped(-0.3, 0.3,  0.10, roll=math.pi),
        ]

        for i, goal in enumerate(goals, start=1):
            with self.subTest(goal_index=i):
                self._send_goal_and_wait(goal)
                self._assert_pose_reached(goal)
