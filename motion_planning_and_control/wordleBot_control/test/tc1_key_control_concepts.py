"""
Test Case 1 — Key Control Concepts
Validates: P (Pass), C (Credit), D (Distinction) criteria

Sub-tests:
  TC1.1  Basic point-to-point movement                    (P)
  TC1.2  Optimised path planning via cost function        (C)
  TC1.3  Gripper open/close                               (C)
  TC1.4  Mission control: Stop / Resume / Abort           (C)
  TC1.5  Collision detection and replanning               (D)
  TC1.6  Integration: basic pick and place                (D)

Prerequisites (must be running before colcon test):
  - UR robot driver:  ros2 launch ur_robot_driver ur_control.launch.py ...
  - MoveIt stack:     ros2 launch ur_moveit_config ur_moveit.launch.py ...
  - Control node launched via tc1_key_control_concepts.launch.py

Run with:
  colcon test --packages-select wordleBot_control --pytest-args -k tc1
  colcon test-result --verbose

  Run TC1.1 in isolation:
  python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_1 -s -v
"""

import math
import os
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException


# ---------------------------------------------------------------------------
# Tolerances (shared across all TC1 tests)
# ---------------------------------------------------------------------------
POSITION_TOLERANCE_M    = 0.005   # 5 mm
ORIENTATION_TOLERANCE_DEG = 5.0   # degrees
MOTION_TIMEOUT_S        = 60.0    # max wait per waypoint
STOP_TIMEOUT_S          = 2.0     # arm must halt within this time of Stop cmd
ABORT_TIMEOUT_S         = 30.0    # arm must reach home within this time of Abort cmd
GRIPPER_TIMEOUT_S       = 2.0     # gripper must respond within this time


# ---------------------------------------------------------------------------
# Helpers  (shared utilities reused across sub-tests)
# ---------------------------------------------------------------------------

def _make_pose_stamped(x: float, y: float, z: float,
                       roll: float = math.pi, pitch: float = 0.0,
                       yaw: float = 0.0,
                       frame_id: str = "world") -> PoseStamped:
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

def _read_last_joint_state(bag_path: str):
    """Return the last JointState message recorded in a bag, or None on failure."""
    try:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        last_msg = None
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == '/joint_states':
                last_msg = deserialize_message(data, JointState)
        return last_msg
    except Exception as exc:
        print(f"[TC1.1] Warning: could not read joint states from bag: {exc}")
        return None



class TestKeyControlConcepts(unittest.TestCase):
    """Test Case 1: Key Control Concepts (validates P, C, D criteria)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("tc1_key_control_concepts_node")

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

        # TODO (TC1.3): create gripper command publisher once topic is defined
        # cls.gripper_pub = cls.node.create_publisher(<GripperMsg>, "/wordle_bot/gripper_cmd", 10)

        # TODO (TC1.3): subscribe to gripper state feedback once topic is defined
        # cls.gripper_state = None
        # cls.node.create_subscription(<GripperState>, "/wordle_bot/gripper_state", cls._on_gripper_state, 10)

        # TODO (TC1.4): create mission control command publisher once topic is defined
        # cls.mission_pub = cls.node.create_publisher(<MissionCmd>, "/wordle_bot/mission_cmd", 10)

        # TODO (TC1.4): subscribe to mission control state once topic is defined
        # cls.mission_state = None
        # cls.node.create_subscription(<MissionState>, "/wordle_bot/mission_state", cls._on_mission_state, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_motion_complete(cls, msg: Bool):
        if msg.data:
            cls.motion_complete = True

    # TODO (TC1.3): implement _on_gripper_state callback once message type is known
    # @classmethod
    # def _on_gripper_state(cls, msg):
    #     cls.gripper_state = msg

    # TODO (TC1.4): implement _on_mission_state callback once message type is known
    # @classmethod
    # def _on_mission_state(cls, msg):
    #     cls.mission_state = msg

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _send_goal_and_wait(self, pose: PoseStamped) -> None:
        """Publish a goal and block until /wordle_bot/motion_complete fires."""
        TestKeyControlConcepts.motion_complete = False
        time.sleep(0.5)  # subscriber handshake
        pose.header.stamp = self.node.get_clock().now().to_msg()
        self.goal_pub.publish(pose)
        self.node.get_logger().info(
            f"Published goal: ({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
        )
        deadline = time.time() + MOTION_TIMEOUT_S
        while not TestKeyControlConcepts.motion_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"Motion did not complete within {MOTION_TIMEOUT_S}s "
                    f"for goal ({pose.pose.position.x:.3f}, "
                    f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
                )

    def _get_ee_transform(self):
        """Look up the current world → tool0 transform from /tf."""
        deadline = time.time() + 5.0
        while time.time() < deadline:
            # Spin first to populate the TF buffer, then do an instant lookup.
            # Using a timeout inside lookup_transform blocks this thread and
            # prevents the spin_once calls needed to receive TF callbacks.
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(
                    "world", "tool0",
                    rclpy.time.Time(),
                )
            except TransformException:
                pass
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
    # TC1.1 — Basic Point-to-Point Movement  (validates P)
    # -----------------------------------------------------------------------

    def test_tc1_1_basic_point_to_point(self):
        """
        Validates: P — basic trajectory generation and EE tracking to a single pose.

        Procedure:
          - Publish a single goal to /wordle_bot/goal_pose.
          - Wait for /wordle_bot/motion_complete = true.
          - Assert EE pose within tolerance via TF lookup.

        Pass: EE within 5 mm / 5° of goal; motion_complete=true within 60 s.
        Fail: Tolerance exceeded, timeout, or planning/execution error.
        """
        goal = _make_pose_stamped(0.3, 0.25, 0.25, roll=math.pi)

        # Start bag recording: capture joint states, TF, and motion_complete signal
        bag_dir = tempfile.mkdtemp(prefix='tc1_1_')
        bag_path = os.path.join(bag_dir, 'tc1_1')
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path,
             '/joint_states', '/tf', '/wordle_bot/motion_complete'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1.0)  # allow the recorder to initialise before sending the goal

        start_time = time.time()
        self._send_goal_and_wait(goal)
        execution_time = time.time() - start_time

        print(f"Waiting for motion to complete... (timeout in {MOTION_TIMEOUT_S} s)")

        while not TestKeyControlConcepts.motion_complete:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        print("Motion complete signal received. Stopping bag recording...") 

        bag_proc.terminate()
        bag_proc.wait()
        print(f"[TC1.1] Bag recorded to: {bag_path}")

        # EE pose: use live TF buffer (composed world→tool0 chain).
        # The bag /tf only has direct parent→child transforms, not the composed
        # world→tool0 result, so we must query the live buffer here.
        tf_result = self._get_ee_transform()
        t = tf_result.transform.translation
        q = tf_result.transform.rotation

        # Joint angles: read final state from bag
        last_js = _read_last_joint_state(bag_path)

        pos_err = math.sqrt(
            (t.x - goal.pose.position.x) ** 2 +
            (t.y - goal.pose.position.y) ** 2 +
            (t.z - goal.pose.position.z) ** 2
        )
        orient_err = _quat_angle_diff_deg(q, goal.pose.orientation)

        passed = True
        try:
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
        except AssertionError:
            passed = False
            raise
        finally:
            # Print final EE pose
            print(
                f"\n[TC1.1] Final EE pose:  "
                f"x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}  "
                f"qx={q.x:.4f}  qy={q.y:.4f}  qz={q.z:.4f}  qw={q.w:.4f}"
            )

            # Print joint angles (from bag)
            if last_js:
                angle_str = '  '.join(
                    f"{n}={math.degrees(a):.1f}°"
                    for n, a in zip(last_js.name, last_js.position)
                )
                print(f"[TC1.1] Final joints:   {angle_str}")
            else:
                print("[TC1.1] Final joints:   (not available — bag read failed)")

            # Print execution time
            print(f"[TC1.1] Execution time: {execution_time:.2f} s")

            # Print overall result
            print(f"[TC1.1] Result:         {'PASS' if passed else 'FAIL'}")

    # -----------------------------------------------------------------------
    # TC1.2 — Optimised Path Planning  (validates C)
    # -----------------------------------------------------------------------

    def test_tc1_2_optimised_path_planning(self):
        """
        Validates: C — controller selects the most efficient trajectory via cost function.

        Procedure:
          - Publish a goal with multiple valid IK solutions from a fixed start state.
          - Extract the full joint trajectory from /joint_states in the bag.
          - Compute total joint displacement: Σ|Δq| across all 6 joints.
          - Compute the minimum-displacement IK baseline for the same goal/start.

        Pass: Executed displacement ≤ 1.5× the minimum-IK baseline.
        Fail: Displacement exceeds 1.5× baseline (no meaningful optimisation).
        """
        goal = _make_pose_stamped(-0.2, 0.3, 0.15, roll=math.pi)

        # TODO: record start joint state from /joint_states
        # start_joint_state = ...

        # TODO: send goal and wait for completion
        # self._send_goal_and_wait(goal)

        # TODO: extract joint trajectory from bag or live /joint_states subscription
        # trajectory_points = ...  # list of joint position vectors

        # TODO: compute total joint displacement from trajectory_points
        # total_displacement = sum(sum(abs(q1 - q0) for q0, q1 in zip(p, trajectory_points[i+1]))
        #                          for i, p in enumerate(trajectory_points[:-1]))

        # TODO: compute minimum-displacement IK baseline
        # (call IK solver with the same start state and goal, pick the lowest-cost solution)
        # min_displacement_baseline = ...

        # TODO: assert displacement ratio within threshold
        # self.assertLessEqual(total_displacement, 1.5 * min_displacement_baseline,
        #     "Path displacement exceeds 1.5× minimum-IK baseline — cost function may not be optimising")

    # -----------------------------------------------------------------------
    # TC1.3 — Gripper Open/Close  (validates C)
    # -----------------------------------------------------------------------

    def test_tc1_3_gripper_open_close(self):
        """
        Validates: C — gripper control is functional.

        Procedure:
          - Send a gripper open command via gripper command topic (TBD).
          - Wait up to 2 s for state feedback to confirm open.
          - Send a gripper close command.
          - Wait up to 2 s for state feedback to confirm closed.

        Pass: State feedback confirms open within 2 s, then closed within 2 s.
        Fail: State does not change, timeout exceeded, or no feedback received.

        NOTE: Gripper command/state topic names TBD — update once interface is implemented.
        """
        # TODO: define open and close command messages once message type is known
        # open_cmd  = <GripperMsg>(position=OPEN_POSITION)
        # close_cmd = <GripperMsg>(position=CLOSED_POSITION)

        # TODO: publish open command and wait for gripper state feedback to confirm open
        # self.gripper_pub.publish(open_cmd)
        # deadline = time.time() + GRIPPER_TIMEOUT_S
        # while self.gripper_state is None or not _gripper_is_open(self.gripper_state):
        #     rclpy.spin_once(self.node, timeout_sec=0.05)
        #     self.assertLess(time.time(), deadline, "Gripper did not open within 2 s")

        # TODO: publish close command and wait for gripper state feedback to confirm closed
        # self.gripper_pub.publish(close_cmd)
        # deadline = time.time() + GRIPPER_TIMEOUT_S
        # while self.gripper_state is None or not _gripper_is_closed(self.gripper_state):
        #     rclpy.spin_once(self.node, timeout_sec=0.05)
        #     self.assertLess(time.time(), deadline, "Gripper did not close within 2 s")

    # -----------------------------------------------------------------------
    # TC1.4 — Mission Control: Stop / Resume / Abort  (validates C)
    # -----------------------------------------------------------------------

    def test_tc1_4_mission_control_stop_resume_abort(self):
        """
        Validates: C — functioning Start/Stop/Resume/Abort framework.

        Procedure (Stop/Resume):
          - Start a 3-waypoint sequence.
          - After waypoint 1, send Stop — verify arm halts within 2 s.
          - Send Resume — verify arm continues from the halted pose.
          - Allow sequence to complete.

        Procedure (Abort):
          - Begin a new 3-waypoint sequence.
          - Send Abort mid-motion — verify arm returns to safe home within 30 s.

        Pass: Arm halts within 2 s of Stop; resumes from correct state; returns home within 30 s of Abort.
        Fail: Arm does not halt, resumes from wrong pose, or fails to return home.

        NOTE: Mission control topic names TBD — update once interface is implemented.
        """
        waypoints = [
            _make_pose_stamped( 0.3,  0.25, 0.25, roll=math.pi),
            _make_pose_stamped(-0.2,  0.3,  0.15, roll=math.pi),
            _make_pose_stamped( 0.2,  0.25, 0.20, roll=math.pi),
        ]

        # --- Stop / Resume sub-test ---

        # TODO: send Start command to begin waypoint sequence
        # self.mission_pub.publish(START_CMD)

        # TODO: wait for waypoint 1 to complete, then send Stop
        # self._send_goal_and_wait(waypoints[0])
        # stop_time = time.time()
        # self.mission_pub.publish(STOP_CMD)

        # TODO: record EE pose at the moment Stop was issued
        # pose_at_stop = self._get_ee_transform()

        # TODO: wait STOP_TIMEOUT_S and verify arm is no longer moving
        # (compare EE pose before and after the timeout window)
        # time.sleep(STOP_TIMEOUT_S)
        # pose_after_stop = self._get_ee_transform()
        # <assert EE displacement < threshold — arm has halted>

        # TODO: send Resume and verify arm continues from the halted pose (not from start)
        # self.mission_pub.publish(RESUME_CMD)
        # self._send_goal_and_wait(waypoints[1])
        # self._assert_pose_reached(waypoints[1])
        # self._send_goal_and_wait(waypoints[2])
        # self._assert_pose_reached(waypoints[2])

        # --- Abort sub-test ---

        # TODO: restart sequence and send Abort mid-motion
        # self.mission_pub.publish(START_CMD)
        # self._send_goal_and_wait(waypoints[0])
        # self.mission_pub.publish(ABORT_CMD)

        # TODO: wait and verify arm returns to safe home position within ABORT_TIMEOUT_S
        # deadline = time.time() + ABORT_TIMEOUT_S
        # while not _at_home_position(self._get_ee_transform()):
        #     rclpy.spin_once(self.node, timeout_sec=0.1)
        #     self.assertLess(time.time(), deadline, f"Arm did not return home within {ABORT_TIMEOUT_S} s after Abort")

    # -----------------------------------------------------------------------
    # TC1.5 — Collision Detection and Replanning  (validates D)
    # -----------------------------------------------------------------------

    def test_tc1_5_collision_detection_and_replanning(self):
        """
        Validates: D — waypoint movement with collision detection and path replanning.

        Procedure:
          - Add a collision box directly between waypoint 1 and waypoint 2 via planning scene.
          - Publish a 3-waypoint sequence.
          - Verify the planner generates a path that avoids the obstacle.
          - Verify all 3 waypoints are reached within tolerance.

        Pass: All waypoints reached; no trajectory point intersects the collision box.
        Fail: Planning failure, waypoint not reached, or path passes through obstacle.
        """
        waypoints = [
            _make_pose_stamped( 0.3,  0.25, 0.25, roll=math.pi),
            _make_pose_stamped(-0.2,  0.3,  0.15, roll=math.pi),
            _make_pose_stamped( 0.2,  0.25, 0.20, roll=math.pi),
        ]

        # TODO: add a collision box to the planning scene between waypoint 0 and waypoint 1
        # obstacle = CollisionObject()
        # obstacle.id = "tc1_5_test_obstacle"
        # obstacle.header.frame_id = "ur_base_link"
        # <set shape: box 0.15 x 0.15 x 0.3 m, positioned at midpoint between wp0 and wp1>
        # <publish obstacle to /planning_scene or use MoveIt PlanningSceneInterface>

        # TODO: send each waypoint in sequence and assert each is reached
        # for i, wp in enumerate(waypoints):
        #     with self.subTest(waypoint=i + 1):
        #         self._send_goal_and_wait(wp)
        #         self._assert_pose_reached(wp)

        # TODO: verify executed trajectory did not intersect the obstacle
        # (extract trajectory from bag; for each joint configuration compute FK;
        #  check EE and all links against obstacle bounding box)
        # <assert no collision>

        # TODO: remove the obstacle from the planning scene after test
        # <remove obstacle>

    # -----------------------------------------------------------------------
    # TC1.6 — Integration: Basic Pick and Place  (validates D)
    # -----------------------------------------------------------------------

    def test_tc1_6_integration_basic_pick_and_place(self):
        """
        Validates: D (integration) — all P/C/D criteria working together:
        waypoint movement, collision avoidance, gripper control, mission control.

        Procedure:
          - Set up collision scene (floor + sensor guard + one obstacle between pick and place).
          - Execute full pick-and-place sequence:
              pre-pick → descend → gripper close → lift → travel (avoid obstacle) → descend → gripper open
          - Issue Stop mid-travel, then Resume — verify mission control holds during pick-and-place.

        Pass: All poses reached within tolerance; gripper feedback confirms pick and place;
              path avoids obstacle; Stop/Resume behaves correctly.
        Fail: Any step fails, gripper does not actuate, path intersects obstacle,
              or Stop/Resume does not function.
        """
        pre_pick_pose = _make_pose_stamped( 0.3,  0.20, 0.15, roll=math.pi)
        pick_pose     = _make_pose_stamped( 0.3,  0.20, 0.05, roll=math.pi)
        lift_pose     = _make_pose_stamped( 0.3,  0.20, 0.20, roll=math.pi)
        place_pose    = _make_pose_stamped(-0.2,  0.25, 0.05, roll=math.pi)

        # TODO: add obstacle between pick and place locations in the planning scene
        # <set up obstacle — same pattern as TC1.5>

        # TODO: move to pre-pick pose
        # self._send_goal_and_wait(pre_pick_pose)
        # self._assert_pose_reached(pre_pick_pose)

        # TODO: descend to pick pose
        # self._send_goal_and_wait(pick_pose)
        # self._assert_pose_reached(pick_pose)

        # TODO: command gripper close (pick)
        # self.gripper_pub.publish(CLOSE_CMD)
        # <wait for gripper closed confirmation>

        # TODO: lift to clearance height
        # self._send_goal_and_wait(lift_pose)
        # self._assert_pose_reached(lift_pose)

        # TODO: travel toward place location and issue Stop mid-travel, then Resume
        # self.mission_pub.publish(STOP_CMD)
        # <assert arm halts within STOP_TIMEOUT_S>
        # self.mission_pub.publish(RESUME_CMD)

        # TODO: descend to place pose
        # self._send_goal_and_wait(place_pose)
        # self._assert_pose_reached(place_pose)

        # TODO: command gripper open (place)
        # self.gripper_pub.publish(OPEN_CMD)
        # <wait for gripper open confirmation>

        # TODO: verify gripper feedback confirms pick and place occurred correctly

        # TODO: remove obstacle from planning scene
        # <remove obstacle>
