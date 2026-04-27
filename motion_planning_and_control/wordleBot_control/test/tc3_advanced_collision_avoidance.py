"""
Test Case 3 — Advanced Collision Avoidance
Validates: Perfect criteria

Sub-tests:
  TC3.1  Active obstacle avoidance: static obstacle injected mid-execution  (Perfect)
  TC3.2  Continuous moving obstacle avoidance                                (Perfect)
  TC3.3  Complete 6-DOF pick and place at arbitrary poses                    (Perfect)

Prerequisites (must be running before colcon test):
  - UR robot driver:  ros2 launch ur_robot_driver ur_control.launch.py ...
  - MoveIt stack:     ros2 launch ur_moveit_config ur_moveit.launch.py ...
  - Control node launched via tc3_advanced_collision_avoidance.launch.py

Run with:
  colcon test --packages-select wordleBot_control --pytest-args -k tc3
  colcon test-result --verbose
"""

import math
import threading
import time
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
POSITION_TOLERANCE_M      = 0.005  # 5 mm
ORIENTATION_TOLERANCE_DEG = 5.0    # degrees
MOTION_TIMEOUT_S          = 60.0   # max wait per waypoint

# TC3.1 — mid-execution injection
INJECTION_DELAY_S         = 2.0    # how long after motion starts before injecting obstacle
REPLAN_TIMEOUT_S          = 15.0   # replanning must complete within this window

# TC3.2 — moving obstacle
OBSTACLE_UPDATE_HZ        = 5.0    # frequency of obstacle position updates (Hz)
MOVING_OBSTACLE_TIMEOUT_S = 60.0   # arm must reach goal despite moving obstacle


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_pose_stamped(x: float, y: float, z: float,
                       roll: float = math.pi, pitch: float = 0.0,
                       yaw: float = 0.0,
                       frame_id: str = "ur_base_link") -> PoseStamped:
    """Build a PoseStamped from (x, y, z, roll, pitch, yaw) in the given frame."""
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
    dot = min(dot, 1.0)
    return math.degrees(2.0 * math.acos(dot))


# ---------------------------------------------------------------------------
# Test fixture
# ---------------------------------------------------------------------------

@pytest.mark.launch(fixture=launch_testing.fixtures.launch_context_with_nodes)
class TestAdvancedCollisionAvoidance(unittest.TestCase):
    """Test Case 3: Advanced Collision Avoidance (validates Perfect criteria)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("tc3_advanced_collision_avoidance_node")

        # Goal publisher
        cls.goal_pub = cls.node.create_publisher(
            PoseStamped, "/wordle_bot/goal_pose", 10
        )

        # Motion complete subscriber
        cls.motion_complete = False
        cls.node.create_subscription(
            Bool,
            "/wordle_bot/motion_complete",
            cls._on_motion_complete,
            10,
        )

        # TF buffer for EE pose lookups
        cls.tf_buffer   = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

        # TODO (TC3.1/TC3.2): create planning scene publisher to inject collision objects at runtime
        # from moveit_msgs.msg import PlanningScene
        # cls.scene_pub = cls.node.create_publisher(PlanningScene, "/planning_scene", 10)

        # TODO (TC3.1): subscribe to execute_trajectory feedback to detect replanning events
        # cls.trajectory_execution_count = 0
        # cls.node.create_subscription(<ExecuteTrajectoryActionFeedback>,
        #     "/execute_trajectory/feedback", cls._on_trajectory_feedback, 10)

        # TODO (TC3.3): create gripper command publisher once topic is defined
        # cls.gripper_pub = cls.node.create_publisher(<GripperMsg>, "/wordle_bot/gripper_cmd", 10)

        # TODO (TC3.3): subscribe to gripper state feedback once topic is defined
        # cls.gripper_state = None
        # cls.node.create_subscription(<GripperState>, "/wordle_bot/gripper_state", cls._on_gripper_state, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_motion_complete(cls, msg: Bool):
        if msg.data:
            cls.motion_complete = True

    # TODO (TC3.1): implement trajectory feedback callback to count replan events
    # @classmethod
    # def _on_trajectory_feedback(cls, msg):
    #     cls.trajectory_execution_count += 1

    # TODO (TC3.3): implement gripper state callback
    # @classmethod
    # def _on_gripper_state(cls, msg):
    #     cls.gripper_state = msg

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _send_goal_and_wait(self, pose: PoseStamped) -> None:
        """Publish a goal and block until /wordle_bot/motion_complete fires."""
        TestAdvancedCollisionAvoidance.motion_complete = False
        time.sleep(0.5)
        pose.header.stamp = self.node.get_clock().now().to_msg()
        self.goal_pub.publish(pose)
        self.node.get_logger().info(
            f"Published goal: ({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
        )
        deadline = time.time() + MOTION_TIMEOUT_S
        while not TestAdvancedCollisionAvoidance.motion_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"Motion did not complete within {MOTION_TIMEOUT_S}s "
                    f"for goal ({pose.pose.position.x:.3f}, "
                    f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
                )

    def _get_ee_transform(self):
        """Look up the current tool0 → ur_base_link transform from /tf."""
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
        """Validate EE transform matches the goal within defined tolerances."""
        tf       = self._get_ee_transform()
        t        = tf.transform.translation
        q_actual = tf.transform.rotation
        q_goal   = goal.pose.orientation

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
    # TC3.1 — Static Obstacle Injected Mid-Execution  (validates Perfect)
    # -----------------------------------------------------------------------

    def test_tc3_1_static_obstacle_mid_execution(self):
        """
        Validates: Perfect — if a hazard enters the environment during execution,
        the controller detects it and re-generates a safe path.

        Procedure:
          - Begin execution of a trajectory to a distant goal (expected duration > 5 s).
          - After INJECTION_DELAY_S seconds, inject a collision box into the planned path.
          - Verify the controller replans without crashing or stalling.
          - Verify the arm reaches the goal within REPLAN_TIMEOUT_S of injection.

        Pass: Second trajectory execution event detected after injection; arm reaches goal
              without collision; replanning completes within 15 s.
        Fail: Arm continues on invalidated path, collides with obstacle, or stalls > 15 s.
        """
        goal_pose = _make_pose_stamped(-0.30, 0.25, 0.20, roll=math.pi)

        # TODO: reset trajectory execution count before starting
        # TestAdvancedCollisionAvoidance.trajectory_execution_count = 0
        # executions_before_injection = 0

        # TODO: start goal execution in a background thread so we can inject mid-motion
        # def _execute():
        #     self._send_goal_and_wait(goal_pose)
        # exec_thread = threading.Thread(target=_execute, daemon=True)
        # exec_thread.start()

        # TODO: wait INJECTION_DELAY_S then inject obstacle directly in the planned path
        # time.sleep(INJECTION_DELAY_S)
        # executions_before_injection = self.trajectory_execution_count
        # injection_time = time.time()
        #
        # obstacle = CollisionObject()
        # obstacle.id = "tc3_1_mid_execution_obstacle"
        # obstacle.header.frame_id = "ur_base_link"
        # <set shape: box 0.20 x 0.20 x 0.40 m, positioned 70% along the arm's current path to goal>
        # self.scene_pub.publish(<planning scene with obstacle added>)

        # TODO: wait for replanning to complete (second execution event detected)
        # deadline = time.time() + REPLAN_TIMEOUT_S
        # while self.trajectory_execution_count <= executions_before_injection:
        #     rclpy.spin_once(self.node, timeout_sec=0.1)
        #     self.assertLess(time.time(), deadline,
        #         f"Replanning did not occur within {REPLAN_TIMEOUT_S} s of obstacle injection")

        # TODO: wait for goal to be reached
        # exec_thread.join(timeout=MOTION_TIMEOUT_S)
        # self._assert_pose_reached(goal_pose)

        # TODO: verify no executed trajectory segment intersects the obstacle
        # (extract from bag or live joint_state log; compute FK for each point; check against obstacle bbox)
        # <assert no collision>

        # TODO: remove obstacle from planning scene
        # <remove obstacle>

    # -----------------------------------------------------------------------
    # TC3.2 — Continuous Moving Obstacle Avoidance  (validates Perfect)
    # -----------------------------------------------------------------------

    def test_tc3_2_continuous_moving_obstacle(self):
        """
        Validates: Perfect — active collision avoidance against a hazard that is
        itself moving; the controller continuously maintains a safe path.

        Procedure:
          - Begin a 2-waypoint trajectory.
          - At OBSTACLE_UPDATE_HZ Hz, continuously update a collision object's position
            so it always intersects the current planned path (simulated moving obstacle).
          - Verify the controller continuously replans to maintain a safe path.
          - Verify the arm reaches the final goal within MOVING_OBSTACLE_TIMEOUT_S.

        Pass: Arm reaches goal; no executed segment intersects obstacle at corresponding timestamp;
              multiple replanning events visible.
        Fail: Arm collides with moving obstacle, fails to reach goal, or produces no replanning events.
        """
        waypoints = [
            _make_pose_stamped( 0.30,  0.25, 0.20, roll=math.pi),
            _make_pose_stamped(-0.30,  0.25, 0.20, roll=math.pi),
        ]

        # TODO: define moving obstacle trajectory
        # The obstacle sweeps in an arc that always intersects the arm's planned straight-line path.
        # obstacle_positions = [
        #     (0.0 + 0.3 * math.cos(t * 0.5), 0.25, 0.2 + 0.1 * math.sin(t * 0.5))
        #     for t in [i / OBSTACLE_UPDATE_HZ for i in range(int(MOVING_OBSTACLE_TIMEOUT_S * OBSTACLE_UPDATE_HZ))]
        # ]

        # TODO: start the moving obstacle publisher in a background thread
        # self._stop_obstacle_thread = False
        # def _publish_moving_obstacle():
        #     idx = 0
        #     while not self._stop_obstacle_thread and idx < len(obstacle_positions):
        #         x, y, z = obstacle_positions[idx]
        #         <build CollisionObject at (x, y, z)>
        #         self.scene_pub.publish(<planning scene update>)
        #         time.sleep(1.0 / OBSTACLE_UPDATE_HZ)
        #         idx += 1
        # obstacle_thread = threading.Thread(target=_publish_moving_obstacle, daemon=True)
        # obstacle_thread.start()

        # TODO: execute the 2-waypoint trajectory
        # for i, wp in enumerate(waypoints):
        #     with self.subTest(waypoint=i + 1):
        #         self._send_goal_and_wait(wp)
        #         self._assert_pose_reached(wp)

        # TODO: stop the obstacle thread
        # self._stop_obstacle_thread = True
        # obstacle_thread.join(timeout=5.0)

        # TODO: verify multiple replanning events occurred during execution
        # self.assertGreater(self.trajectory_execution_count, len(waypoints),
        #     "No replanning events detected — controller may not be responding to moving obstacle")

        # TODO: verify no executed trajectory segment intersected obstacle at corresponding timestamp
        # <extract joint_state log and obstacle position log; correlate by timestamp; assert no collision>

        # TODO: clean up obstacle from planning scene
        # <remove obstacle>

    # -----------------------------------------------------------------------
    # TC3.3 — Complete 6-DOF Pick and Place at Arbitrary Poses  (validates Perfect)
    # -----------------------------------------------------------------------

    def test_tc3_3_complete_6dof_pick_and_place(self):
        """
        Validates: Perfect — complete robust pick and place control given any
        6 DOF pose (X, Y, Z, roll, pitch, yaw), including non-trivial approach angles.

        Procedure:
          - Define 3 pick/place pose pairs with non-trivial roll and pitch (tilted approach angles).
          - Active collision scene present throughout.
          - Execute full pick-and-place at each pose pair.

        Pass: All 3 pick and place poses reached within 5 mm / 5°; gripper succeeds at each;
              IK solves without failure for all 6-DOF poses; paths avoid collision objects.
        Fail: Any pose out of tolerance, IK failure, gripper failure, or collision with obstacle.
        """
        # 3 pick/place pairs with non-trivial roll and pitch (tilted surfaces)
        pick_place_pairs = [
            (
                _make_pose_stamped( 0.25,  0.20, 0.08,
                                    roll=math.radians(180 + 20), pitch=math.radians(-15), yaw=math.radians(  0)),
                _make_pose_stamped(-0.20,  0.25, 0.10,
                                    roll=math.pi,                  pitch=0.0,              yaw=math.radians( 45)),
            ),
            (
                _make_pose_stamped( 0.15, -0.18, 0.06,
                                    roll=math.radians(180 - 30), pitch=math.radians( 10), yaw=math.radians( 90)),
                _make_pose_stamped( 0.28,  0.15, 0.12,
                                    roll=math.radians(180 + 15), pitch=math.radians(-20), yaw=math.radians(-30)),
            ),
            (
                _make_pose_stamped(-0.18,  0.25, 0.05,
                                    roll=math.radians(180 + 25), pitch=math.radians(-25), yaw=math.radians(180)),
                _make_pose_stamped( 0.22, -0.15, 0.10,
                                    roll=math.radians(180 - 10), pitch=math.radians( 15), yaw=math.radians( 60)),
            ),
        ]

        for pair_idx, (pick_pose, place_pose) in enumerate(pick_place_pairs):
            with self.subTest(pair=pair_idx + 1):
                # TODO: add obstacle between this pick and place pair in the planning scene
                # <obstacle setup — similar to TC1.5>

                # TODO: approach pick pose (lift to clearance above pick location)
                # approach_pick = _make_pose_stamped(
                #     pick_pose.pose.position.x,
                #     pick_pose.pose.position.y,
                #     pick_pose.pose.position.z + 0.10,
                #     <same orientation as pick_pose>
                # )
                # self._send_goal_and_wait(approach_pick)
                # self._assert_pose_reached(approach_pick)

                # TODO: descend to pick pose
                # self._send_goal_and_wait(pick_pose)
                # self._assert_pose_reached(pick_pose)

                # TODO: command gripper close (pick)
                # self.gripper_pub.publish(CLOSE_CMD)
                # <wait for gripper closed confirmation>

                # TODO: lift back to clearance height
                # self._send_goal_and_wait(approach_pick)

                # TODO: transport to above place pose
                # approach_place = _make_pose_stamped(
                #     place_pose.pose.position.x,
                #     place_pose.pose.position.y,
                #     place_pose.pose.position.z + 0.10,
                #     <same orientation as place_pose>
                # )
                # self._send_goal_and_wait(approach_place)

                # TODO: descend to place pose
                # self._send_goal_and_wait(place_pose)
                # self._assert_pose_reached(place_pose)

                # TODO: command gripper open (place)
                # self.gripper_pub.publish(OPEN_CMD)
                # <wait for gripper open confirmation>

                # TODO: verify gripper feedback confirms successful pick and place
                # TODO: verify no IK failures were logged during this pair
                # TODO: remove obstacle from planning scene
                pass
