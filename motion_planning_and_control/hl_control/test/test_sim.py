#!/usr/bin/env python3
"""
TC2.1 simulation test publisher.

Simulates the perception and solver nodes by reading a board YAML and
publishing once to both gameboard_state and word_request. Use this to
test hl_control_node in isolation without the full robotic system.

Publishes to:
  /perception/gameboard_state  (hl_control/GameboardState) — latched
  /hl_control/word_request     (std_msgs/String)            — latched

Usage:
  python3 test/test_sim.py --ros-args \
    -p config_path:=config/tc2_1_board.yaml \
    -p word:=CRANE

After hl_control_node logs its task sequence, trigger execution manually:
  ros2 topic pub /wordle_bot/start_mission std_msgs/msg/Bool "data: true" --once

YAML format (see config/tc2_1_board.yaml):
  word: CRANE
  letters:
    - letter: C
      object_id: C_object_1
      x: -0.448
      y:  0.072
      z:  0.025
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0
    ...
"""

import os
import yaml
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from hl_control.msg import GameboardState, LetterObject


class TestEnvPublisher(Node):
    def __init__(self):
        super().__init__('test_env_publisher')

        self.declare_parameter('config_path', '')
        self.declare_parameter('word', '')
        self.declare_parameter('publish_delay', 2.0)

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        override_word = self.get_parameter('word').get_parameter_value().string_value
        delay = self.get_parameter('publish_delay').get_parameter_value().double_value

        if not config_path or not os.path.exists(config_path):
            self.get_logger().fatal(f'config_path not found: "{config_path}"')
            raise RuntimeError(f'config_path not found: {config_path}')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self._word = override_word.strip().upper() if override_word else config.get('word', '')
        self._letters_cfg = config.get('letters', [])

        latched_qos = QoSProfile(depth=1,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._board_pub = self.create_publisher(
            GameboardState, '/perception/gameboard_state', latched_qos)
        self._word_pub = self.create_publisher(
            String, '/hl_control/word_request', latched_qos)

        # Publish after a short delay to allow subscribers to connect
        self._timer = self.create_timer(delay, self._publish_once)
        self.get_logger().info(
            f'TestEnvPublisher ready — will publish "{self._word}" in {delay:.1f}s.')

    def _publish_once(self) -> None:
        board_msg = GameboardState()
        for item in self._letters_cfg:
            lo = LetterObject()
            lo.letter = str(item.get('letter', '')).strip().upper()
            lo.object_id = str(item.get('object_id', f"{lo.letter}_object_1"))

            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(item.get('x', 0.0))
            ps.pose.position.y = float(item.get('y', 0.0))
            ps.pose.position.z = float(item.get('z', 0.025))
            ps.pose.orientation.x = float(item.get('qx', 0.0))
            ps.pose.orientation.y = float(item.get('qy', 0.0))
            ps.pose.orientation.z = float(item.get('qz', 0.0))
            ps.pose.orientation.w = float(item.get('qw', 1.0))
            lo.pose = ps
            board_msg.letters.append(lo)

        self._board_pub.publish(board_msg)
        self.get_logger().info(
            f'Published gameboard_state with {len(board_msg.letters)} letter(s).')

        word_msg = String()
        word_msg.data = self._word
        self._word_pub.publish(word_msg)
        self.get_logger().info(f'Published word_request: "{self._word}".')

        # One-shot: cancel the timer after publishing
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TestEnvPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
