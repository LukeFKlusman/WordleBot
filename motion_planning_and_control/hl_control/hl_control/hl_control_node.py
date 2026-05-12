#!/usr/bin/env python3 
"""
HLControlNode — high-level Wordle-bot decision maker.

Subscribes to:
  /hl_control/word_request      (std_msgs/String)       — target word from solver/test
  /perception/gameboard_state   (hl_control/GameboardState) — perceived letter poses

Publishes to:
  /perception/letter_objects    (wordlebot_control/PickPlaceTask) — one msg per task
  /wordle_bot/add_collision_object (moveit_msgs/CollisionObject) — letters into MoveIt scene
  /wordle_bot/start_mission     (std_msgs/Bool)          — arm execution after queuing

Waits until both word and board state have arrived before solving.
Re-solves on each new word request (board state is latched from last received).
"""

import sys
import os

# Allow sibling scripts (rl_task_optimiser.py) to be imported whether this node
# is run installed (lib/hl_control/) or from source.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from hl_control.msg import GameboardState
from wordlebot_control.msg import PickPlaceTask

from rl_task_optimiser import RLTaskOptimiser

LETTER_CUBE_SIZE = 0.05     # 50 mm cube for collision objects
SOLVE_STAGE = 3             # C3 masking handles both C2-like and C3-like boards


class HLControlNode(Node):
    def __init__(self):
        super().__init__('hl_control_node')

        self.declare_parameter('model_path', '')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._optimiser = RLTaskOptimiser(model_path or None)

        self._pending_word: str | None = None
        self._board_letters: list[dict] | None = None

        # Latched QoS for board state so test publisher can publish once
        latched_qos = QoSProfile(depth=1,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._word_sub = self.create_subscription(
            String,
            '/hl_control/word_request',
            self._word_callback,
            latched_qos,
        )
        self._board_sub = self.create_subscription(
            GameboardState,
            '/perception/gameboard_state',
            self._board_callback,
            latched_qos,
        )

        self._task_pub = self.create_publisher(PickPlaceTask, '/perception/letter_objects', 10)
        self._collision_pub = self.create_publisher(
            CollisionObject, '/wordle_bot/add_collision_object', 10)
        self._start_pub = self.create_publisher(Bool, '/wordle_bot/start_mission', 10)

        self.get_logger().info('HLControlNode ready — waiting for word request and board state.')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _word_callback(self, msg: String) -> None:
        word = msg.data.strip().upper()
        if len(word) != 5 or not word.isalpha():
            self.get_logger().warn(f'Received invalid word "{msg.data}" — must be 5 letters.')
            return
        self.get_logger().info(f'Word request received: {word}')
        self._pending_word = word
        self._try_solve()

    def _board_callback(self, msg: GameboardState) -> None:
        self._board_letters = []
        for lo in msg.letters:
            self._board_letters.append({
                'letter':    lo.letter.strip().upper(),
                'object_id': lo.object_id,
                'x':         lo.pose.pose.position.x,
                'y':         lo.pose.pose.position.y,
                'z':         lo.pose.pose.position.z,
                'qx':        lo.pose.pose.orientation.x,
                'qy':        lo.pose.pose.orientation.y,
                'qz':        lo.pose.pose.orientation.z,
                'qw':        lo.pose.pose.orientation.w,
            })
        self.get_logger().info(
            f'Board state received: {len(self._board_letters)} letter(s).')
        self._try_solve()

    # ------------------------------------------------------------------
    # Solve pipeline
    # ------------------------------------------------------------------

    def _try_solve(self) -> None:
        if self._pending_word is None or self._board_letters is None:
            return
        word = self._pending_word
        letters = self._board_letters
        self._pending_word = None  # consume — next word request triggers a fresh solve

        self.get_logger().info(
            f'Solving: word="{word}" with {len(letters)} letter(s) on board.')

        self._add_letters_to_scene(letters)

        try:
            sequence = self._optimiser.solve(word, letters)
        except Exception as e:
            self.get_logger().error(f'RL solver failed: {e}')
            return

        if not sequence:
            self.get_logger().warn('RL solver returned an empty task sequence.')
            return

        self.get_logger().info(f'Task sequence ({len(sequence)} step(s)):')
        for task in sequence:
            self.get_logger().info(
                f"  Step {task['step']}: {task['description']} | "
                f"pick=({task['pick_x']:.3f},{task['pick_y']:.3f},{task['pick_z']:.3f}) "
                f"place=({task['place_x']:.3f},{task['place_y']:.3f},{task['place_z']:.3f}) "
                f"id='{task['object_id']}'"
            )

        self._publish_tasks(sequence)

        start_msg = Bool()
        start_msg.data = True
        self._start_pub.publish(start_msg)
        self.get_logger().info('All tasks queued — start_mission published.')

    def _add_letters_to_scene(self, letters: list[dict]) -> None:
        for item in letters:
            co = CollisionObject()
            co.id = item['object_id']
            co.header.frame_id = 'world'
            co.header.stamp = self.get_clock().now().to_msg()
            co.operation = CollisionObject.ADD

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [LETTER_CUBE_SIZE, LETTER_CUBE_SIZE, LETTER_CUBE_SIZE]
            co.primitives.append(box)

            pose = Pose()
            pose.position.x = item['x']
            pose.position.y = item['y']
            pose.position.z = item['z']
            pose.orientation.x = item.get('qx', 0.0)
            pose.orientation.y = item.get('qy', 0.0)
            pose.orientation.z = item.get('qz', 0.0)
            pose.orientation.w = item.get('qw', 1.0)
            co.primitive_poses.append(pose)

            self._collision_pub.publish(co)

        self.get_logger().info(
            f'Published {len(letters)} collision object(s) to MoveIt scene.')

    def _publish_tasks(self, sequence: list[dict]) -> None:
        for task in sequence:
            msg = PickPlaceTask()

            msg.pick_pose.header.frame_id = 'world'
            msg.pick_pose.header.stamp = self.get_clock().now().to_msg()
            msg.pick_pose.pose.position.x = task['pick_x']
            msg.pick_pose.pose.position.y = task['pick_y']
            msg.pick_pose.pose.position.z = task['pick_z']
            msg.pick_pose.pose.orientation.x = task['pick_qx']
            msg.pick_pose.pose.orientation.y = task['pick_qy']
            msg.pick_pose.pose.orientation.z = task['pick_qz']
            msg.pick_pose.pose.orientation.w = task['pick_qw']

            msg.place_pose.position.x = task['place_x']
            msg.place_pose.position.y = task['place_y']
            msg.place_pose.position.z = task['place_z']
            msg.place_pose.orientation.w = 1.0   # identity

            msg.object_id = task['object_id']

            self._task_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HLControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
