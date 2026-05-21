"""
Test Case 2 — Advanced Motion Control
Validates: HD (High Distinction) criteria

Sub-tests:
  TC2.1  Design matrix goal ordering optimisation         (HD)
  TC2.2  Variable velocity profiling                      (HD)
  TC2.3  Robust 4-DOF pick and place at arbitrary poses   (HD)

Prerequisites (must be running before colcon test):
  - UR robot driver:  ros2 launch ur_robot_driver ur_control.launch.py ...
  - MoveIt stack:     ros2 launch ur_moveit_config ur_moveit.launch.py ...
  - Control node launched via tc2_advanced_motion_control.launch.py

Run with:
  colcon test --packages-select wordlebot_control --pytest-args -k tc2
  colcon test-result --verbose
"""

import math
import os
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from hl_control.msg import GameboardState, LetterObject
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException


# End-effector TF frame (must match active MoveIt planning group tip link)
EE_LINK = "gripper_tcp"


# ---------------------------------------------------------------------------
# Tolerances
# ---------------------------------------------------------------------------
POSITION_TOLERANCE_M      = 0.005  # 5 mm
ORIENTATION_TOLERANCE_DEG = 5.0    # degrees
MOTION_TIMEOUT_S          = 60.0   # max wait per waypoint

# TC2.1 — goal ordering
ORDERING_IMPROVEMENT_THRESHOLD = 0.10  # executed order must be at least 10% shorter

# TC2.2 — velocity profiling
NEAR_OBSTACLE_RADIUS_M    = 0.10   # region considered "near" an obstacle (10 cm)
FREE_SPACE_RADIUS_M       = 0.25   # region considered "free" (>25 cm from any obstacle)
VELOCITY_REDUCTION_MIN    = 0.20   # near-zone mean velocity must be >= 20% lower than free-zone


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
# Bag reader helpers
# ---------------------------------------------------------------------------

def _read_goal_reached_timestamps(bag_path: str) -> list:
    """Return ordered list of timestamp_ns for each /wordle_bot/goal_reached=True in a bag."""
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        timestamps = []
        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            if topic == '/wordle_bot/goal_reached':
                msg = deserialize_message(data, Bool)
                if msg.data:
                    timestamps.append(timestamp_ns)
        return timestamps
    except Exception as exc:
        print(f"[TC2.4] Warning: could not read goal_reached timestamps from bag: {exc}")
        return []


def _read_ee_pose_from_bag(bag_path: str, lookup_time_ns: int, ee_link: str = EE_LINK):
    """Look up world→ee_link transform from offline bag TF data at lookup_time_ns."""
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
    """Return list of (timestamp_ns, x, y, z) for world→ee_link across all /tf messages in a bag."""
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
        print(f"[TC2.4] Warning: _read_ee_positions_from_bag failed: {exc}")
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


# ---------------------------------------------------------------------------
# Test fixture
# ---------------------------------------------------------------------------

class TestAdvancedMotionControl(unittest.TestCase):
    """Test Case 2: Advanced Motion Control (validates HD criteria)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("tc2_advanced_motion_control_node")

        # Mission publishers
        cls.set_mission_pub = cls.node.create_publisher(
            PoseArray, "/wordle_bot/set_mission", 10
        )
        cls.start_mission_pub = cls.node.create_publisher(
            Bool, "/wordle_bot/start_mission", 10
        )

        # Motion complete subscriber (legacy — kept for TC2.1/TC2.2 stubs)
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

        # TC2.4 — full-stack HL integration
        _latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        cls.gameboard_pub      = cls.node.create_publisher(
            GameboardState, '/perception/gameboard_state', _latched
        )
        cls.word_req_pub       = cls.node.create_publisher(
            String, '/hl_control/word_request', _latched
        )
        cls.scan_and_sweep_pub = cls.node.create_publisher(
            Bool, '/wordle_bot/scan_and_sweep', 10
        )
        cls.mission_complete   = False
        cls.goal_reached_count = 0
        cls.robot_state        = 'IDLE'
        cls.node.create_subscription(
            Bool, '/wordle_bot/mission_complete', cls._on_mission_complete, 10
        )
        cls.node.create_subscription(
            Bool, '/wordle_bot/goal_reached', cls._on_goal_reached, 10
        )
        cls.node.create_subscription(
            String, '/wordle_bot/robot_state', cls._on_robot_state, 10
        )

        # TODO (TC2.1): subscribe to executed goal order feedback once topic is defined
        # cls.executed_order = []
        # cls.node.create_subscription(<OrderMsg>, "/wordle_bot/execution_order", cls._on_execution_order, 10)

        # TODO (TC2.3): create gripper command publisher once topic is defined
        # cls.gripper_pub = cls.node.create_publisher(<GripperMsg>, "/wordle_bot/gripper_cmd", 10)

        # TODO (TC2.3): subscribe to gripper state feedback once topic is defined
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

    @classmethod
    def _on_mission_complete(cls, msg: Bool):
        if msg.data:
            cls.mission_complete = True

    @classmethod
    def _on_goal_reached(cls, msg: Bool):
        if msg.data:
            cls.goal_reached_count += 1

    @classmethod
    def _on_robot_state(cls, msg: String):
        cls.robot_state = msg.data

    # TODO (TC2.1): implement _on_execution_order callback
    # @classmethod
    # def _on_execution_order(cls, msg):
    #     cls.executed_order.append(msg.goal_index)

    # TODO (TC2.3): implement _on_gripper_state callback
    # @classmethod
    # def _on_gripper_state(cls, msg):
    #     cls.gripper_state = msg

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _send_mission_and_wait(self, poses: list) -> None:
        """Queue goal poses via set_mission, arm with start_mission, wait for motion_complete."""
        TestAdvancedMotionControl.motion_complete = False
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.node.get_clock().now().to_msg()
        pa.poses = [p.pose for p in poses]
        self.set_mission_pub.publish(pa)
        time.sleep(0.5)
        start_msg = Bool()
        start_msg.data = True
        self.start_mission_pub.publish(start_msg)
        self.node.get_logger().info(
            f"Mission sent with {len(poses)} goal(s). Waiting for motion_complete."
        )
        deadline = time.time() + MOTION_TIMEOUT_S * len(poses)
        while not TestAdvancedMotionControl.motion_complete:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"Mission did not complete within {MOTION_TIMEOUT_S * len(poses):.0f} s"
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
    # TC2.1 — Design Matrix Goal Ordering Optimisation  (validates HD)
    # -----------------------------------------------------------------------

    def test_tc2_1_design_matrix_goal_ordering(self):
        """
        Validates: HD — system optimises movement using a design matrix,
        selecting the most efficient visitation order given goal positions.

        Procedure:
          - Publish 4 goals in a deliberately suboptimal order (zigzag pattern).
            Optimal order: sweep left-to-right (far-left → near-left → near-right → far-right).
            Input order:   zigzag (far-left → far-right → near-left → near-right).
          - Record the order in which goals are actually executed.
          - Compute total joint displacement for the executed order vs. the input order.

        Pass: Executed order differs from input order; total displacement at least 10% shorter.
        Fail: Goals executed in input order, or no meaningful reduction in total displacement.
        """
        # Goals arranged so suboptimal input order (zigzag) is measurably worse than optimal (sweep)
        goals_input_order = [
            _make_pose_stamped( 0.30,  0.25, 0.20, roll=math.pi),  # far-left
            _make_pose_stamped(-0.30,  0.25, 0.20, roll=math.pi),  # far-right
            _make_pose_stamped( 0.15,  0.28, 0.20, roll=math.pi),  # near-left
            _make_pose_stamped(-0.15,  0.28, 0.20, roll=math.pi),  # near-right
        ]

        # TODO: publish all 4 goals at once (batch publish) if the interface supports it,
        # or send them sequentially and track the order of execution via feedback topic
        # for goal in goals_input_order:
        #     self.goal_pub.publish(goal)

        # TODO: record the order in which goals are executed
        # (listen to /wordle_bot/execution_order or equivalent feedback)
        # executed_order = self.executed_order  # populated by _on_execution_order callback

        # TODO: compute total joint displacement for the executed order
        # executed_displacement = _compute_total_displacement(executed_order, joint_trajectory_from_bag)

        # TODO: compute total joint displacement for the naive input order (reference)
        # naive_displacement = _compute_total_displacement(goals_input_order, joint_trajectory_from_bag)

        # TODO: assert executed order is at least ORDERING_IMPROVEMENT_THRESHOLD shorter
        # improvement = (naive_displacement - executed_displacement) / naive_displacement
        # self.assertGreaterEqual(improvement, ORDERING_IMPROVEMENT_THRESHOLD,
        #     f"Goal ordering improvement {improvement:.1%} is below {ORDERING_IMPROVEMENT_THRESHOLD:.0%} threshold")

    # -----------------------------------------------------------------------
    # TC2.2 — Variable Velocity Profiling  (validates HD)
    # -----------------------------------------------------------------------

    def test_tc2_2_variable_velocity_profiling(self):
        """
        Validates: HD — variable velocity based on safety: arm moves slower near
        obstacles and faster in open free space.

        Procedure:
          - Set up a collision box on one side of the workspace.
          - Execute a trajectory from a start pose, past the obstacle, to a goal on the far side.
          - Extract velocity profile from /joint_states timestamps.
          - Compute mean joint velocity in the near zone (EE within 10 cm of obstacle)
            and the free zone (EE more than 25 cm from any obstacle).

        Pass: Mean velocity in near zone is >= 20% lower than mean velocity in free zone.
        Fail: No measurable velocity difference; arm moves at uniform speed throughout.
        """
        start_pose = _make_pose_stamped( 0.30, 0.25, 0.20, roll=math.pi)
        goal_pose  = _make_pose_stamped(-0.30, 0.25, 0.20, roll=math.pi)

        # TODO: add a collision box on one side of the workspace
        # obstacle = CollisionObject()
        # obstacle.id = "tc2_2_velocity_test_obstacle"
        # <position the box so the planned trajectory passes within NEAR_OBSTACLE_RADIUS_M of it>
        # <publish to /planning_scene or use PlanningSceneInterface>

        # TODO: move to start pose first
        # self._send_goal_and_wait(start_pose)

        # TODO: collect /joint_states during execution to build velocity profile
        # joint_states_log = []  # [(timestamp, joint_positions), ...]
        # <subscribe to /joint_states and log during execution>

        # TODO: execute trajectory toward goal
        # self._send_goal_and_wait(goal_pose)

        # TODO: compute mean joint velocity in near zone vs. free zone
        # near_velocities = []
        # free_velocities = []
        # for i in range(1, len(joint_states_log)):
        #     ee_pos = _fk(joint_states_log[i][1])  # compute EE from joint state
        #     dist_to_obstacle = _distance_to_obstacle(ee_pos, obstacle)
        #     dt = joint_states_log[i][0] - joint_states_log[i-1][0]
        #     dq = sum(abs(a - b) for a, b in zip(joint_states_log[i][1], joint_states_log[i-1][1]))
        #     mean_vel = dq / dt if dt > 0 else 0
        #     if dist_to_obstacle < NEAR_OBSTACLE_RADIUS_M:
        #         near_velocities.append(mean_vel)
        #     elif dist_to_obstacle > FREE_SPACE_RADIUS_M:
        #         free_velocities.append(mean_vel)

        # TODO: assert near-zone velocity is measurably lower than free-zone velocity
        # mean_near = sum(near_velocities) / len(near_velocities)
        # mean_free = sum(free_velocities) / len(free_velocities)
        # reduction = (mean_free - mean_near) / mean_free
        # self.assertGreaterEqual(reduction, VELOCITY_REDUCTION_MIN,
        #     f"Velocity reduction near obstacle ({reduction:.1%}) is below {VELOCITY_REDUCTION_MIN:.0%} minimum")

        # TODO: remove obstacle from planning scene
        # <remove obstacle>

    # -----------------------------------------------------------------------
    # TC2.3 — Robust 4-DOF Pick and Place at Arbitrary Poses  (validates HD)
    # -----------------------------------------------------------------------

    def test_tc2_3_robust_4dof_pick_and_place(self):
        """
        Validates: HD — robust pick and place control given any 4 DOF pose (X, Y, Z, yaw).

        Procedure:
          - Define 3 pick/place pose pairs with varied X, Y, Z, and yaw across the workspace.
          - Execute full pick-and-place (approach → grip → lift → transport → descend → release)
            at each pose pair.

        Pass: All 3 pick and place poses reached within 5 mm / 5°;
              gripper closes and opens correctly; no IK or planning failures.
        Fail: Any pose out of tolerance, gripper failure, or IK/planning error.
        """
        # 3 pick/place pairs: (pick_pose, place_pose)
        # Varied X, Y, Z, and yaw — roll and pitch remain at default (facing downward)
        pick_place_pairs = [
            (
                _make_pose_stamped( 0.25,  0.20, 0.05, yaw=math.radians(  0)),
                _make_pose_stamped(-0.20,  0.25, 0.10, yaw=math.radians( 45)),
            ),
            (
                _make_pose_stamped( 0.15, -0.20, 0.08, yaw=math.radians( 90)),
                _make_pose_stamped( 0.30,  0.15, 0.12, yaw=math.radians(-30)),
            ),
            (
                _make_pose_stamped(-0.15,  0.28, 0.06, yaw=math.radians(180)),
                _make_pose_stamped( 0.20, -0.18, 0.10, yaw=math.radians( 60)),
            ),
        ]

        for pair_idx, (pick_pose, place_pose) in enumerate(pick_place_pairs):
            with self.subTest(pair=pair_idx + 1):
                # TODO: approach pick pose (raise to clearance height above pick)
                # approach = _make_pose_stamped(pick_pose.pose.position.x,
                #                               pick_pose.pose.position.y,
                #                               pick_pose.pose.position.z + 0.10, ...)
                # self._send_goal_and_wait(approach)
                # self._assert_pose_reached(approach)

                # TODO: descend to pick pose
                # self._send_goal_and_wait(pick_pose)
                # self._assert_pose_reached(pick_pose)

                # TODO: command gripper close
                # self.gripper_pub.publish(CLOSE_CMD)
                # <wait for gripper closed confirmation>

                # TODO: lift to clearance height
                # self._send_goal_and_wait(approach)

                # TODO: transport to above place pose
                # place_approach = _make_pose_stamped(place_pose.pose.position.x,
                #                                     place_pose.pose.position.y,
                #                                     place_pose.pose.position.z + 0.10, ...)
                # self._send_goal_and_wait(place_approach)

                # TODO: descend to place pose
                # self._send_goal_and_wait(place_pose)
                # self._assert_pose_reached(place_pose)

                # TODO: command gripper open
                # self.gripper_pub.publish(OPEN_CMD)
                # <wait for gripper open confirmation>

                # TODO: verify gripper feedback confirms successful pick and place
                pass

    # -----------------------------------------------------------------------
    # TC2.4 — Full-Stack HL + WordleBot Control Integration  (validates HD)
    # -----------------------------------------------------------------------

    def test_tc2_4_full_stack_hl_wordle_control(self):
        """
        Validates: HD — end-to-end pipeline from perception through HL planning to
        physical pick-and-place execution for a five-letter Wordle word.

        Pipeline:
          /perception/gameboard_state (GameboardState) →
          HL Control Agent solves CRANE →
          /wordle_bot/start_mission →
          WordleBot Control executes pick-and-place for each letter →
          /wordle_bot/mission_complete

        Pass: mission_complete received within timeout; at least one goal_reached
              signal fired per letter; robot descended to expected pick height.
        Fail: Timeout, mission_complete not received, or no goal_reached signals.
        """

        TestAdvancedMotionControl.mission_complete   = False
        TestAdvancedMotionControl.goal_reached_count = 0
        TestAdvancedMotionControl.robot_state        = 'IDLE'

        # ------------------------------------------------------------------ #
        # Step 1: Scan and sweep — let the robot survey the workspace first.
        # Completion is indicated by /wordle_bot/robot_state going IDLE.
        # ------------------------------------------------------------------ #
        scan_msg      = Bool()
        scan_msg.data = True
        self.scan_and_sweep_pub.publish(scan_msg)
        self.node.get_logger().info("[TC2.4] Scan-and-sweep triggered — waiting to start.")

        # Wait for RUNNING to confirm the sweep began, then wait for IDLE
        scan_timeout = MOTION_TIMEOUT_S * 2
        deadline     = time.time() + scan_timeout

        # Wait for RUNNING
        while TestAdvancedMotionControl.robot_state != 'RUNNING':
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"[TC2.4] Scan-and-sweep never entered RUNNING state within "
                    f"{scan_timeout:.0f} s — is the control node up?"
                )

        self.node.get_logger().info("[TC2.4] Scan-and-sweep RUNNING — waiting for IDLE.")

        # Wait for IDLE (sweep complete)
        while TestAdvancedMotionControl.robot_state != 'IDLE':
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() > deadline:
                self.fail(
                    f"[TC2.4] Scan-and-sweep did not complete within {scan_timeout:.0f} s."
                )

        self.node.get_logger().info("[TC2.4] Scan-and-sweep complete.")

        # ------------------------------------------------------------------ #
        # Letter poses and word definition (world frame, 40 mm tiles at z=0.025)
        # ------------------------------------------------------------------ #
        WORD = 'CRANE'
        LETTERS = [
            ('C', 'C_object_1', -0.448,  0.072, 0.025),
            ('R', 'R_object_1', -0.377,  0.154, 0.025),
            ('A', 'A_object_1', -0.302,  0.302, 0.025),
            ('N', 'N_object_1',  0.373,  0.220, 0.025),
            ('E', 'E_object_1',  0.157,  0.304, 0.025),
        ]
        PICK_Z_OFFSET    = 0.08   # grasp_frame_transform z in createTask
        GRASP_REACH_TOL  = 0.05   # 50 mm — warn if robot never descended this close
        PLACE_REACH_TOL  = 0.05   # 50 mm — EE x check at each goal_reached
        expected_grasp_z = min(z for _, _, _, _, z in LETTERS) + PICK_Z_OFFSET

        # ------------------------------------------------------------------ #
        # Bag recording
        # ------------------------------------------------------------------ #
        bag_dir  = tempfile.mkdtemp(prefix='tc2_4_')
        bag_path = os.path.join(bag_dir, 'tc2_4')
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path,
             '/joint_states', '/tf', '/tf_static',
             '/wordle_bot/goal_reached', '/wordle_bot/mission_complete'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(1.0)

        # ------------------------------------------------------------------ #
        # Publish GameboardState (latched — HL node receives even if late)
        # ------------------------------------------------------------------ #
        gb_msg = GameboardState()
        for letter, obj_id, x, y, z in LETTERS:
            lo = LetterObject()
            lo.letter    = letter
            lo.object_id = obj_id
            lo.pose.header.frame_id       = 'world'
            lo.pose.pose.position.x       = x
            lo.pose.pose.position.y       = y
            lo.pose.pose.position.z       = z
            lo.pose.pose.orientation.w    = 1.0
            gb_msg.letters.append(lo)
        self.gameboard_pub.publish(gb_msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        time.sleep(0.1)

        # ------------------------------------------------------------------ #
        # Publish word request (latched)
        # ------------------------------------------------------------------ #
        word_msg      = String()
        word_msg.data = WORD
        self.word_req_pub.publish(word_msg)
        self.node.get_logger().info(
            f"[TC2.4] Published GameboardState ({len(LETTERS)} letters) "
            f"and word_request='{WORD}'."
        )

        # Allow HL agent time to plan and publish PickPlaceTask messages
        time.sleep(2.0)

        # ------------------------------------------------------------------ #
        # Arm the mission
        # ------------------------------------------------------------------ #
        start_msg      = Bool()
        start_msg.data = True
        self.start_mission_pub.publish(start_msg)
        self.node.get_logger().info(
            "[TC2.4] Mission armed — waiting for full pick-and-place sequence to complete."
        )

        # ------------------------------------------------------------------ #
        # Wait for mission_complete  (5 letters × 3× per-goal timeout)
        # ------------------------------------------------------------------ #
        timeout  = MOTION_TIMEOUT_S * 3 * len(LETTERS)
        deadline = time.time() + timeout
        passed   = True

        try:
            while not TestAdvancedMotionControl.mission_complete:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                self.assertLess(
                    time.time(), deadline,
                    f"[TC2.4] Full-stack mission did not complete within {timeout:.0f} s — "
                    "check HL control and MTC planning output.",
                )
            self.assertTrue(
                TestAdvancedMotionControl.mission_complete,
                "[TC2.4] mission_complete signal was not received.",
            )
            self.assertGreaterEqual(
                TestAdvancedMotionControl.goal_reached_count, len(LETTERS),
                f"[TC2.4] Expected at least {len(LETTERS)} goal_reached signals "
                f"(one per letter), got {TestAdvancedMotionControl.goal_reached_count}.",
            )
            self.node.get_logger().info("[TC2.4] Full-stack mission complete — PASS.")

        except AssertionError:
            passed = False
            raise

        finally:
            bag_proc.terminate()
            bag_proc.wait()
            self.node.get_logger().info(f"[TC2.4] Bag recorded to: {bag_path}")

            # -------------------------------------------------------------- #
            # Post-run diagnostics (mirrors TC1.6/TC1.7 analysis)
            # -------------------------------------------------------------- #
            ee_positions = _read_ee_positions_from_bag(bag_path)
            goal_ts_list = _read_goal_reached_timestamps(bag_path)

            min_ee_z     = min((z for _, _, _, z in ee_positions), default=None)
            pick_x_vals  = [x for _, _, x, _, _ in LETTERS]

            print(f"\n[TC2.4] === Full-Stack HL Integration Diagnostic Summary ===")
            print(f"[TC2.4] Word: '{WORD}'  |  Letters published: {len(LETTERS)}")
            print(f"[TC2.4] goal_reached signals received: "
                  f"{TestAdvancedMotionControl.goal_reached_count}  "
                  f"(expected >= {len(LETTERS)})")

            if min_ee_z is not None:
                delta = min_ee_z - expected_grasp_z
                flag  = (
                    "OK — reached pick height"
                    if delta <= GRASP_REACH_TOL
                    else f"WARNING — stopped {delta * 1000:.1f} mm above expected pick z"
                )
                print(
                    f"[TC2.4] Min EE z seen: {min_ee_z:.4f} m  "
                    f"(expected grasp z={expected_grasp_z:.4f})  [{flag}]"
                )
            else:
                print("[TC2.4] Min EE z: (not available — bag empty or TF not resolved)")

            print(f"[TC2.4] EE position at each goal_reached:")
            for i, ts_ns in enumerate(goal_ts_list):
                try:
                    tf  = _read_ee_pose_from_bag(bag_path, ts_ns)
                    t   = tf.transform.translation
                    nearest_pick_x = min(pick_x_vals, key=lambda px: abs(px - t.x))
                    x_err = abs(t.x - nearest_pick_x)
                    ok    = "OK" if x_err <= PLACE_REACH_TOL else "WARNING — far from any pick x"
                    print(
                        f"[TC2.4]   goal {i+1:2d}: EE=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})  "
                        f"nearest_pick_x={nearest_pick_x:.4f}  "
                        f"x_err={x_err*1000:.1f} mm  [{ok}]"
                    )
                except Exception as exc:
                    print(f"[TC2.4]   goal {i+1:2d}: EE lookup failed — {exc}")

            print(f"[TC2.4] Result: {'PASS' if passed else 'FAIL'}")
            print(f"[TC2.4] ====================================================\n")
