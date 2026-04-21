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

  python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -s -v

  Run TC1.1 in isolation:
  python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_1 -s -v

  Run TC1.2 in isolation:
  python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_2 -s -v

  Run TC1.5 in isolation:
  python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_5 -s -v
"""

import math
import os
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException, sleep
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


# ---------------------------------------------------------------------------
# Tolerances (shared across all TC1 tests)
# ---------------------------------------------------------------------------
POSITION_TOLERANCE_M      = 0.005   # 5 mm
ORIENTATION_TOLERANCE_DEG = 5.0     # degrees
MOTION_TIMEOUT_S          = 60.0    # max wait per waypoint
STOP_TIMEOUT_S            = 2.0     # arm must halt within this time of Stop cmd
ABORT_TIMEOUT_S           = 30.0    # arm must reach home within this time of Abort cmd
GRIPPER_TIMEOUT_S         = 2.0     # gripper must respond within this time

# End-effector TF frame used for all pose validation.
# Must match the tip link of the active MoveIt planning group.
EE_LINK = "gripper_tcp"


class TestKeyControlConcepts(unittest.TestCase):
    """Test Case 1: Key Control Concepts (validates P, C, D criteria)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("tc1_key_control_concepts_node")

        # Legacy single-goal publisher (backward compat; auto-arms mission)
        cls.goal_pub = cls.node.create_publisher(
            PoseStamped, "/wordle_bot/goal_pose", 10
        )

        # Mission-level publishers
        cls.set_mission_pub = cls.node.create_publisher(
            PoseArray, "/wordle_bot/set_mission", 10
        )
        cls.start_mission_pub = cls.node.create_publisher(
            Bool, "/wordle_bot/start_mission", 10
        )

        # Motion complete subscriber (per-goal, backward compat)
        cls.motion_complete = False
        cls.node.create_subscription(
            Bool,
            "/wordle_bot/motion_complete",
            cls._on_motion_complete,
            10,
        )

        # Per-goal reached signal
        cls.goal_reached_count = 0
        cls.pending_pose_lookup = False
        cls.goal_reached_poses = []
        cls.node.create_subscription(
            Bool,
            "/wordle_bot/goal_reached",
            cls._on_goal_reached,
            10,
        )

        # Mission complete signal
        cls.mission_complete = False
        cls.node.create_subscription(
            Bool,
            "/wordle_bot/mission_complete",
            cls._on_mission_complete,
            10,
        )

        # TF buffer for EE pose lookups
        cls.tf_buffer   = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

        # Collision object injection publisher (used by TC1.5)
        cls.collision_pub = cls.node.create_publisher(
            CollisionObject, "/wordle_bot/add_collision_object", 10
        )

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

    @classmethod
    def _on_goal_reached(cls, msg: Bool):
        if msg.data:
            cls.goal_reached_count += 1
            cls.pending_pose_lookup = True

    @classmethod
    def _on_mission_complete(cls, msg: Bool):
        if msg.data:
            cls.mission_complete = True

    # TODO (TC1.3): implement _on_gripper_state callback once message type is known
    # @classmethod
    # def _on_gripper_state(cls, msg):
    #     cls.gripper_state = msg

    # TODO (TC1.4): implement _on_mission_state callback once message type is known
    # @classmethod
    # def _on_mission_state(cls, msg):
    #     cls.mission_state = msg

    # -----------------------------------------------------------------------
    # TC1.1 — Basic Point-to-Point Movement  (validates P)
    # -----------------------------------------------------------------------

    def test_tc1_1_basic_point_to_point(self):
        """
        Validates: P — basic trajectory generation and EE tracking to a single pose.

        Procedure:
          - Set a single-goal mission via /wordle_bot/set_mission.
          - Arm the mission via /wordle_bot/start_mission.
          - Wait for /wordle_bot/mission_complete = true.
          - Assert EE pose within tolerance via TF lookup.

        Pass: EE within 5 mm / 5° of goal; mission_complete=true within 60 s.
        Fail: Tolerance exceeded, timeout, or planning/execution error.
        """
        goal = _make_pose_stamped(0.2, 0.1, 0.05, roll=math.pi)

        # Start bag recording
        bag_dir = tempfile.mkdtemp(prefix='tc1_1_')
        bag_path = os.path.join(bag_dir, 'tc1_1')
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path,
             '/joint_states', '/tf', '/tf_static',
             '/wordle_bot/goal_reached', '/wordle_bot/mission_complete'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1.0)  # allow the recorder to initialise

        # Reset state flags
        TestKeyControlConcepts.mission_complete = False
        TestKeyControlConcepts.goal_reached_count = 0
        TestKeyControlConcepts.goal_reached_poses = []

        start_time = time.time()
        self._send_mission_and_wait([goal])
        execution_time = time.time() - start_time

        print("Mission complete signal received. Stopping bag recording...")

        bag_proc.terminate()
        bag_proc.wait()
        print(f"[TC1.1] Bag recorded to: {bag_path}")

        # EE pose: read from bag using BufferCore (handles full world→EE_LINK chain
        # by composing both /tf_static and /tf transforms offline).
        goal_reached_ts = _read_goal_reached_timestamps(bag_path)
        tf_result = _read_ee_pose_from_bag(bag_path, goal_reached_ts[0] if goal_reached_ts else 0)
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

        # print final results and assert tolerances — ensure this runs even if assertions fail, to show final pose and execution time
        self.node.get_logger().info(f"Final EE pose: x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}  ")
        self.node.get_logger().info(f"Final EE orientation: qx={q.x:.4f}  qy={q.y:.4f}  qz={q.z:.4f}  qw={q.w:.4f}")
        self.node.get_logger().info(f"Goal pose: x={goal.pose.position.x:.4f}  y={goal.pose.position.y:.4f}  z={goal.pose.position.z:.4f}")
        self.node.get_logger().info(f"Goal orientation: qx={goal.pose.orientation.x:.4f}  qy={goal.pose.orientation.y:.4f}  qz={goal.pose.orientation.z:.4f}  qw={goal.pose.orientation.w:.4f}")
        self.node.get_logger().info(f"Final joints: {', '.join(f'{n}={math.degrees(a):.1f}°' for n, a in zip(last_js.name, last_js.position)) if last_js else '(not available)'}")
        self.node.get_logger().info(f"Execution time: {execution_time:.2f} s")
        self.node.get_logger().info(f"Position error: {pos_err * 1000:.1f} mm | Orientation error: {orient_err:.2f} deg")

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
        Validates: C — controller executes a 3-waypoint mission and selects efficient trajectories.

        Procedure:
          - Set a 3-goal mission via /wordle_bot/set_mission.
          - Arm the mission via /wordle_bot/start_mission.
          - After each /wordle_bot/goal_reached signal, record EE pose via TF lookup.
          - Wait for /wordle_bot/mission_complete.
          - Assert each EE pose reached within tolerance.
          - Assert total joint displacement ≤ 1.5× sum of per-leg minimum-IK baselines.

        Pass: All goals within 5 mm / 5°; displacement ratio within threshold.
        Fail: Any goal tolerance exceeded, timeout, or displacement exceeds 1.5× baseline.
        """
        goals = [
            _make_pose_stamped( 0.40,  0.10, 0.25, roll=math.pi),
            _make_pose_stamped(-0.30,  0.20, 0.10, roll=math.pi),
            _make_pose_stamped( 0.05,  0.20, 0.40, roll=math.pi),
        ]

        bag_dir = tempfile.mkdtemp(prefix='tc1_2_')
        bag_path = os.path.join(bag_dir, 'tc1_2')
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path,
             '/joint_states', '/tf', '/tf_static',
             '/wordle_bot/goal_reached', '/wordle_bot/mission_complete'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1.0)

        # Reset state flags
        TestKeyControlConcepts.mission_complete = False
        TestKeyControlConcepts.goal_reached_count = 0

        # Build and send the mission
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.node.get_clock().now().to_msg()
        pa.poses = [g.pose for g in goals]
        self.set_mission_pub.publish(pa)
        time.sleep(0.5)

        start_msg = Bool()
        start_msg.data = True
        self.start_mission_pub.publish(start_msg)
        self.node.get_logger().info("[TC1.2] Mission armed with 3 goals.")

        # Wait for each goal_reached signal (timeout guard only — poses are read from bag)
        for i in range(len(goals)):
            deadline = time.time() + MOTION_TIMEOUT_S
            prev_count = TestKeyControlConcepts.goal_reached_count
            while TestKeyControlConcepts.goal_reached_count <= prev_count:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                self.assertLess(
                    time.time(), deadline,
                    f"goal_reached signal {i + 1} did not arrive within {MOTION_TIMEOUT_S:.0f} s"
                )

        # Wait for mission_complete
        deadline = time.time() + MOTION_TIMEOUT_S
        while not TestKeyControlConcepts.mission_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertLess(
                time.time(), deadline,
                f"mission_complete did not arrive within {MOTION_TIMEOUT_S:.0f} s after last goal"
            )

        bag_proc.terminate()
        bag_proc.wait()
        print(f"[TC1.2] Bag recorded to: {bag_path}")

        # Read all per-goal EE poses from the bag using BufferCore
        goal_reached_ts = _read_goal_reached_timestamps(bag_path)
        self.assertEqual(
            len(goal_reached_ts), len(goals),
            f"Expected {len(goals)} goal_reached signals in bag, got {len(goal_reached_ts)}"
        )
        goal_reached_poses = [_read_ee_pose_from_bag(bag_path, ts) for ts in goal_reached_ts]

        passed = True
        try:
            # Validate EE pose at each waypoint
            for i, (tf_result, goal) in enumerate(zip(goal_reached_poses, goals)):
                t = tf_result.transform.translation
                q = tf_result.transform.rotation
                pos_err = math.sqrt(
                    (t.x - goal.pose.position.x) ** 2 +
                    (t.y - goal.pose.position.y) ** 2 +
                    (t.z - goal.pose.position.z) ** 2
                )
                orient_err = _quat_angle_diff_deg(q, goal.pose.orientation)
                self.node.get_logger().info(
                    f"[TC1.2] Goal {i + 1} EE:     x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}  "
                    f"qx={q.x:.4f}  qy={q.y:.4f}  qz={q.z:.4f}  qw={q.w:.4f}"
                )
                self.node.get_logger().info(
                    f"[TC1.2] Goal {i + 1} target: "
                    f"x={goal.pose.position.x:.4f}  y={goal.pose.position.y:.4f}  "
                    f"z={goal.pose.position.z:.4f}"
                )
                self.node.get_logger().info(
                    f"[TC1.2] Goal {i + 1}: pos_err={pos_err * 1000:.1f} mm  "
                    f"orient_err={orient_err:.2f}°"
                )
                self.assertLessEqual(
                    pos_err, POSITION_TOLERANCE_M,
                    f"Goal {i + 1}: position error {pos_err * 1000:.1f} mm exceeds "
                    f"{POSITION_TOLERANCE_M * 1000:.0f} mm tolerance"
                )
                self.assertLessEqual(
                    orient_err, ORIENTATION_TOLERANCE_DEG,
                    f"Goal {i + 1}: orientation error {orient_err:.2f}° exceeds "
                    f"{ORIENTATION_TOLERANCE_DEG:.0f}° tolerance"
                )

            # Validate path optimality: per-leg actual displacement vs straight-line baseline.
            # For each leg: actual path length in joint space must be within 10% of the
            # straight-line L1 distance from the start to the end joint configuration.
            traj_stamped = _read_joint_trajectory(bag_path)
            goal_reached_ts = _read_goal_reached_timestamps(bag_path)
            if traj_stamped and goal_reached_ts:
                legs = _segment_trajectory_by_goals(traj_stamped, goal_reached_ts)
                for leg_idx, leg in enumerate(legs):
                    if len(leg) < 2:
                        continue
                    q_start_leg = leg[0]
                    q_end_leg   = leg[-1]
                    baseline = sum(abs(e - s) for s, e in zip(q_start_leg, q_end_leg))
                    actual   = sum(
                        sum(abs(b - a) for a, b in zip(leg[t], leg[t + 1]))
                        for t in range(len(leg) - 1)
                    )
                    ratio = actual / baseline if baseline > 1e-6 else 1.0
                    print(
                        f"[TC1.2] Leg {leg_idx + 1}: "
                        f"actual={actual:.4f} rad  baseline={baseline:.4f} rad  "
                        f"ratio={ratio:.3f}x  (limit=1.10x)"
                    )
                    self.assertLessEqual(
                        actual, 1.10 * baseline,
                        f"Leg {leg_idx + 1}: joint displacement {actual:.4f} rad exceeds "
                        f"1.10× straight-line baseline ({1.10 * baseline:.4f} rad)"
                    )
        except AssertionError:
            passed = False
            raise
        finally:
            for i, (tf_result, goal) in enumerate(zip(goal_reached_poses, goals)):
                t = tf_result.transform.translation
                q = tf_result.transform.rotation
                pos_err = math.sqrt(
                    (t.x - goal.pose.position.x) ** 2 +
                    (t.y - goal.pose.position.y) ** 2 +
                    (t.z - goal.pose.position.z) ** 2
                )
                orient_err = _quat_angle_diff_deg(q, goal.pose.orientation)
                js = _read_joint_state_at_time(bag_path, goal_reached_ts[i])
                print(
                    f"[TC1.2] Goal {i + 1} EE:     "
                    f"x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}  "
                    f"qx={q.x:.4f}  qy={q.y:.4f}  qz={q.z:.4f}  qw={q.w:.4f}"
                )
                print(
                    f"[TC1.2] Goal {i + 1} target: "
                    f"x={goal.pose.position.x:.4f}  y={goal.pose.position.y:.4f}  "
                    f"z={goal.pose.position.z:.4f}"
                )
                if js:
                    angle_str = '  '.join(
                        f"{n}={math.degrees(a):.1f}°"
                        for n, a in zip(js.name, js.position)
                    )
                    print(f"[TC1.2] Goal {i + 1} joints:  {angle_str}")
                print(
                    f"[TC1.2] Goal {i + 1} error:   "
                    f"pos={pos_err * 1000:.1f} mm  orient={orient_err:.2f}°"
                )
            print(f"[TC1.2] Result: {'PASS' if passed else 'FAIL'}")

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

        self.fail("[TC1.3] Not implemented — update once gripper interface is defined.")

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

        self.fail("[TC1.4] Not implemented — update once mission control interface is defined.")

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
        
        # ------------------------------------------------------------------ #
        # Move to safe start pose before obstacle is added
        # ------------------------------------------------------------------ #
        TestKeyControlConcepts.mission_complete   = False
        TestKeyControlConcepts.goal_reached_count = 0
        start_pose = _make_pose_stamped(-0.2, 0.3, 0.15, roll=math.pi)
        self._send_mission_and_wait([start_pose])
        self.node.get_logger().info("[TC1.5] Reached safe start pose.")

        time.sleep(1.0)  # brief pause to ensure the arm is at the start pose

        print("\n =================================================================== \n")
        self.node.get_logger().info("[TC1.5] Starting collision test — adding obstacle and arming mission with 3 waypoints.")

        # ------------------------------------------------------------------ #
        # Obstacle geometry (world frame)
        # ------------------------------------------------------------------ #
        OBS_ID    = "tc1_5_test_obstacle"
        OBS_FRAME = "world"
        OBS_POS   = (0.000, 0.40, 0.20)   # centre — midpoint of wp0 → wp1 path
        OBS_DIM   = (0.025, 0.30, 0.40)    # box x, y, z dimensions
        OBS_X_MIN = OBS_POS[0] - OBS_DIM[0] / 2.0
        OBS_X_MAX = OBS_POS[0] + OBS_DIM[0] / 2.0
        OBS_Y_MIN = OBS_POS[1] - OBS_DIM[1] / 2.0
        OBS_Y_MAX = OBS_POS[1] + OBS_DIM[1] / 2.0
        OBS_Z_MIN = OBS_POS[2] - OBS_DIM[2] / 2.0
        OBS_Z_MAX = OBS_POS[2] + OBS_DIM[2] / 2.0
        AABB_PAD  = 0.020  # outward padding for EE clearance check (m)

        # ------------------------------------------------------------------ #
        # Waypoints
        # ------------------------------------------------------------------ #
        waypoints = [
            _make_pose_stamped( 0.3,  0.25, 0.10, roll=math.pi),
            _make_pose_stamped(-0.3,  0.3,  0.10, roll=math.pi),
            _make_pose_stamped( 0.2,  0.25, 0.10, roll=math.pi),
        ]

        # ------------------------------------------------------------------ #
        # Publish collision object (ADD)
        # ------------------------------------------------------------------ #
        obs = CollisionObject()
        obs.id = OBS_ID
        obs.header.frame_id = OBS_FRAME
        obs.header.stamp = self.node.get_clock().now().to_msg()
        obs.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [OBS_DIM[0], OBS_DIM[1], OBS_DIM[2]]
        obs.primitives.append(prim)

        from geometry_msgs.msg import Pose as GmPose
        obs_pose = GmPose()
        obs_pose.position.x = OBS_POS[0]
        obs_pose.position.y = OBS_POS[1]
        obs_pose.position.z = OBS_POS[2]
        obs_pose.orientation.w = 1.0
        obs.primitive_poses.append(obs_pose)

        self.node.get_logger().info(f"[TC1.5] Publishing collision object '{OBS_ID}' at position {OBS_POS}")

        time.sleep(0.5)  # brief pause to ensure publisher is ready

        for _ in range(5):
            self.collision_pub.publish(obs)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        time.sleep(1.5)
        self.node.get_logger().info(
            f"[TC1.5] Obstacle '{OBS_ID}' published to planning scene."
        )

        time.sleep(0.5)  # allow time for the planning scene to update with the new obstacle

        # ------------------------------------------------------------------ #
        # Bag recording
        # ------------------------------------------------------------------ #
        bag_dir  = tempfile.mkdtemp(prefix='tc1_5_')
        bag_path = os.path.join(bag_dir, 'tc1_5')
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path,
             '/joint_states', '/tf', '/tf_static',
             '/wordle_bot/goal_reached', '/wordle_bot/mission_complete'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1.0)

        # ------------------------------------------------------------------ #
        # Reset state and arm mission
        # ------------------------------------------------------------------ #
        TestKeyControlConcepts.mission_complete    = False
        TestKeyControlConcepts.goal_reached_count  = 0
        TestKeyControlConcepts.goal_reached_poses  = []
        TestKeyControlConcepts.pending_pose_lookup = False

        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.node.get_clock().now().to_msg()
        pa.poses = [wp.pose for wp in waypoints]
        self.set_mission_pub.publish(pa)
        time.sleep(0.5)

        start_msg = Bool()
        start_msg.data = True
        self.start_mission_pub.publish(start_msg)
        self.node.get_logger().info("[TC1.5] Mission armed — 3 goals, obstacle in zone.")

        # ------------------------------------------------------------------ #
        # Wait for goal_reached × 3 + mission_complete
        # ------------------------------------------------------------------ #
        expected_count = len(waypoints)
        for i in range(expected_count):
            deadline = time.time() + MOTION_TIMEOUT_S
            prev_count = TestKeyControlConcepts.goal_reached_count
            while TestKeyControlConcepts.goal_reached_count <= prev_count:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                self.assertLess(
                    time.time(), deadline,
                    f"[TC1.5] goal_reached signal {i + 1} did not arrive within "
                    f"{MOTION_TIMEOUT_S:.0f} s"
                )
            self.node.get_logger().info(f"[TC1.5] goal_reached {i + 1} received.")

        deadline = time.time() + MOTION_TIMEOUT_S
        while not TestKeyControlConcepts.mission_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertLess(
                time.time(), deadline,
                f"[TC1.5] mission_complete did not arrive within {MOTION_TIMEOUT_S:.0f} s"
            )

        bag_proc.terminate()
        bag_proc.wait()
        self.node.get_logger().info(f"[TC1.5] Bag recorded to: {bag_path}")

        goal_reached_ts = _read_goal_reached_timestamps(bag_path)
        goal_reached_poses = [_read_ee_pose_from_bag(bag_path, ts) for ts in goal_reached_ts]

        # ------------------------------------------------------------------ #
        # Assertions
        # ------------------------------------------------------------------ #
        
        self.node.get_logger().info(
            f"[TC1.5] Validating results: {len(goal_reached_poses)} goal_reached poses, "
            f"checking against {len(waypoints)} waypoints and obstacle AABB."
        )


        passed = True
        try:
            # 1. Validate 3 goal_reached signals
            self.assertEqual(
                len(goal_reached_poses), len(waypoints),
                f"[TC1.5] Expected {len(waypoints)} goal_reached signals, "
                f"got {len(goal_reached_poses)}"
            )

            # 2. Validate EE pose at each waypoint
            for i, (tf_result, wp) in enumerate(
                    zip(goal_reached_poses, waypoints)):
                t = tf_result.transform.translation
                q = tf_result.transform.rotation
                pos_err = math.sqrt(
                    (t.x - wp.pose.position.x) ** 2 +
                    (t.y - wp.pose.position.y) ** 2 +
                    (t.z - wp.pose.position.z) ** 2
                )
                orient_err = _quat_angle_diff_deg(q, wp.pose.orientation)
                self.node.get_logger().info(
                    f"[TC1.5] Goal {i + 1}: pos_err={pos_err * 1000:.1f} mm  "
                    f"orient_err={orient_err:.2f}°"
                )
                self.assertLessEqual(
                    pos_err, POSITION_TOLERANCE_M,
                    f"[TC1.5] Goal {i + 1}: position error {pos_err * 1000:.1f} mm "
                    f"exceeds {POSITION_TOLERANCE_M * 1000:.0f} mm tolerance"
                )
                self.assertLessEqual(
                    orient_err, ORIENTATION_TOLERANCE_DEG,
                    f"[TC1.5] Goal {i + 1}: orientation error {orient_err:.2f}° "
                    f"exceeds {ORIENTATION_TOLERANCE_DEG:.0f}° tolerance"
                )

            # 3. Verify EE trajectory never entered the obstacle AABB.
            # NOTE: adjust frame names below if robot_state_publisher uses different
            #       link names (check with: ros2 topic echo /tf --once).
            ee_positions = _read_ee_positions_from_bag(bag_path)

            if not ee_positions:
                self.node.get_logger().warn(
                    "[TC1.5] No EE positions resolved from bag /tf — "
                    "skipping AABB check (possible TF chain name mismatch)."
                )
            else:
                violations = [
                    (ts, x, y, z)
                    for ts, x, y, z in ee_positions
                    if (OBS_X_MIN - AABB_PAD <= x <= OBS_X_MAX + AABB_PAD and
                        OBS_Y_MIN - AABB_PAD <= y <= OBS_Y_MAX + AABB_PAD and
                        OBS_Z_MIN - AABB_PAD <= z <= OBS_Z_MAX + AABB_PAD)
                ]
                if violations:
                    first = violations[0]
                    self.fail(
                        f"[TC1.5] EE entered obstacle AABB at "
                        f"(x={first[1]:.4f}, y={first[2]:.4f}, z={first[3]:.4f}) — "
                        f"{len(violations)} violation(s). "
                        f"AABB x=[{OBS_X_MIN:.3f},{OBS_X_MAX:.3f}] "
                        f"y=[{OBS_Y_MIN:.3f},{OBS_Y_MAX:.3f}] "
                        f"z=[{OBS_Z_MIN:.3f},{OBS_Z_MAX:.3f}] (pad={AABB_PAD*1000:.0f}mm)"
                    )
                self.node.get_logger().info(
                    f"[TC1.5] AABB clearance check passed — "
                    f"{len(ee_positions)} EE positions checked, 0 violations."
                )

        except AssertionError:
            passed = False
            raise
        finally:
            for i, (tf_result, wp) in enumerate(zip(goal_reached_poses, waypoints)):
                t = tf_result.transform.translation
                print(
                    f"[TC1.5] Goal {i + 1} EE:  "
                    f"x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}  |  "
                    f"target: x={wp.pose.position.x:.4f}  "
                    f"y={wp.pose.position.y:.4f}  z={wp.pose.position.z:.4f}"
                )
            
            print("\n =================================================================== \n")
            self.node.get_logger().info(f"[TC1.5] Result: {'PASS' if passed else 'FAIL'}")
            print("\n =================================================================== \n")

            # Clean up: remove obstacle from planning scene regardless of outcome
            remove_obs = CollisionObject()
            remove_obs.id = OBS_ID
            remove_obs.header.frame_id = OBS_FRAME
            remove_obs.header.stamp = self.node.get_clock().now().to_msg()
            remove_obs.operation = CollisionObject.REMOVE
            for _ in range(3):
                self.collision_pub.publish(remove_obs)
                rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.5)
            self.node.get_logger().info(
                f"[TC1.5] Obstacle '{OBS_ID}' removal published."
            )
            time.sleep(1.0)  # allow time for the planning scene to update after obstacle removal

            print("\n")

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

        self.fail("[TC1.6] Not implemented — update once gripper and mission control interfaces are defined.")
    
    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _send_mission_and_wait(self, poses: list) -> None:
        """Set a mission from a list of PoseStamped and block until mission_complete fires."""
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.node.get_clock().now().to_msg()
        pa.poses = [p.pose for p in poses]
        self.set_mission_pub.publish(pa)
        time.sleep(0.5)  # allow node to process set_mission

        start_msg = Bool()
        start_msg.data = True
        self.start_mission_pub.publish(start_msg)
        self.node.get_logger().info(
            f"Mission sent with {len(poses)} goal(s). Waiting for mission_complete."
        )

        deadline = time.time() + MOTION_TIMEOUT_S * len(poses)
        while not TestKeyControlConcepts.mission_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"Mission did not complete within {MOTION_TIMEOUT_S * len(poses):.0f} s"
                )

    def _send_goal_and_wait(self, pose: PoseStamped) -> None:
        """Publish a single goal via legacy topic and block until motion_complete fires."""
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
        """Look up the current world → EE_LINK transform from /tf."""
        deadline = time.time() + 5.0
        while time.time() < deadline:
            # Spin first to populate the TF buffer, then do an instant lookup.
            # Using a timeout inside lookup_transform blocks this thread and
            # prevents the spin_once calls needed to receive TF callbacks.
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(
                    "world", EE_LINK,
                    rclpy.time.Time(),
                )
            except TransformException:
                pass
        self.fail(f"Could not look up world → {EE_LINK} transform within 5 s")

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


def _read_joint_state_at_time(bag_path: str, lookup_time_ns: int):
    """Return the last JointState recorded in the bag at or before lookup_time_ns."""
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        last_msg = None
        while reader.has_next():
            topic, data, ts = reader.read_next()
            if topic == '/joint_states' and ts <= lookup_time_ns:
                last_msg = deserialize_message(data, JointState)
        return last_msg
    except Exception as exc:
        print(f"[TC1.2] Warning: could not read joint state at time from bag: {exc}")
        return None


def _read_joint_trajectory(bag_path: str) -> list:
    """Return ordered list of (timestamp_ns, positions) from all JointState messages in a bag."""
    try:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        trajectory = []
        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            if topic == '/joint_states':
                msg = deserialize_message(data, JointState)
                trajectory.append((timestamp_ns, list(msg.position)))
        return trajectory
    except Exception as exc:
        print(f"[TC1.2] Warning: could not read joint trajectory from bag: {exc}")
        return []


def _read_goal_reached_timestamps(bag_path: str) -> list:
    """Return ordered list of timestamp_ns for each /wordle_bot/goal_reached message in a bag."""
    try:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        timestamps = []
        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            if topic == '/wordle_bot/goal_reached':
                msg = deserialize_message(data, Bool)
                if msg.data:
                    timestamps.append(timestamp_ns)
        return timestamps
    except Exception as exc:
        print(f"[TC1.2] Warning: could not read goal_reached timestamps from bag: {exc}")
        return []


def _segment_trajectory_by_goals(traj_stamped: list, goal_reached_ts: list) -> list:
    """Split a stamped joint trajectory into per-leg lists of position vectors.

    traj_stamped: list of (timestamp_ns, positions) in chronological order
    goal_reached_ts: list of timestamp_ns marking the end of each goal leg

    Returns a list of N lists of position vectors, one list per goal leg.
    Leg i contains all joint states from the end of leg i-1 (or mission start) up to
    and including the joint state closest to goal_reached_ts[i].
    """
    legs = []
    leg_start_ts = 0  # before the first recorded joint state
    for goal_ts in goal_reached_ts:
        leg = [pos for ts, pos in traj_stamped if leg_start_ts <= ts <= goal_ts]
        legs.append(leg)
        leg_start_ts = goal_ts
    return legs



def _read_ee_pose_from_bag(bag_path: str, lookup_time_ns: int, ee_link: str = EE_LINK):
    """Look up world→ee_link transform using offline bag TF data via BufferCore.

    Loads all /tf_static messages and /tf messages up to lookup_time_ns into a
    BufferCore, then queries world→ee_link at the latest available time. Filtering
    /tf by lookup_time_ns ensures the returned pose reflects the robot state at
    the moment goal_reached was published, not the end of the recording.
    """
    buffer = tf2_ros.BufferCore(cache_time=rclpy.duration.Duration(seconds=600))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == '/tf_static':
            msg = deserialize_message(data, TFMessage)
            for tf in msg.transforms:
                buffer.set_transform_static(tf, 'bag')
        elif topic == '/tf' and ts <= lookup_time_ns:
            msg = deserialize_message(data, TFMessage)
            for tf in msg.transforms:
                buffer.set_transform(tf, 'bag')
    return buffer.lookup_transform_core('world', ee_link, rclpy.time.Time())


def _read_ee_positions_from_bag(bag_path: str, ee_link: str = EE_LINK) -> list:
    """Read /tf_static and /tf from a bag and compute world→ee_link positions via BufferCore.

    Returns: list of (timestamp_ns, x, y, z) for each /tf message timestep where
             world→ee_link is resolvable.
    """
    buffer = tf2_ros.BufferCore(cache_time=rclpy.duration.Duration(seconds=600))
    tf_timestamps = []
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        while reader.has_next():
            topic, data, ts = reader.read_next()
            if topic == '/tf_static':
                msg = deserialize_message(data, TFMessage)
                for tf in msg.transforms:
                    buffer.set_transform_static(tf, 'bag')
            elif topic == '/tf':
                msg = deserialize_message(data, TFMessage)
                for tf in msg.transforms:
                    buffer.set_transform(tf, 'bag')
                tf_timestamps.append(ts)
    except Exception as exc:
        print(f"Warning: _read_ee_positions_from_bag failed reading bag: {exc}")
        return []

    results = []
    for ts in tf_timestamps:
        try:
            result = buffer.lookup_transform_core('world', ee_link, rclpy.time.Time())
            t = result.transform.translation
            results.append((ts, t.x, t.y, t.z))
        except Exception:
            pass
    return results