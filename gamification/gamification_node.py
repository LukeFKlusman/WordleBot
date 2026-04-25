#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  gamification_node.py
#
#  ROS2 node for Subsystem 4 — Gamification.
#  Wraps wordle_logic.py in a proper ROS2 node.
#
#  TOPICS SUBSCRIBED:
#    /perception/detections      std_msgs/String  — JSON block list from SS1
#    /mission/state              std_msgs/String  — game start/stop from SS3
#    /gamification/feedback      std_msgs/String  — G/B/I feedback from SS3/voice
#
#  TOPICS PUBLISHED:
#    /gamification/guess         std_msgs/String  — current word guess
#    /gamification/mission_state std_msgs/String  — ordered placement JSON for SS2
#    /diagnostics                std_msgs/String  — board state JSON for SS3 GUI
#
#  SERVICES:
#    /gamification/reset         — resets the game state
#    /gamification/undo          — simple one-step undo
#
#  HOW TO RUN:
#    Terminal 1: ros2 launch realsense2_camera rs_launch.py ...
#    Terminal 2: python3 perception/src/realsense_camera_cnn.py
#    Terminal 3: python3 gamification/gamification_node.py
#    Terminal 4: python3 voice_control/voice_node.py   (optional)
# ─────────────────────────────────────────────────────────────────

import os
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dictionary   import load_dictionary
from wordle_logic import (
    filter_candidates,
    choose_opening_guess,
    choose_best_guess,
    parse_feedback,
)
from constants import GOOD, BAD_POSITION, INCORRECT


class GamificationNode(Node):

    def __init__(self):
        super().__init__('gamification_node')

        dict_path  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
        self.words = load_dictionary(dict_path)
        self.get_logger().info(f'Dictionary loaded: {len(self.words)} words')

        # ── Game state ────────────────────────────────────────────
        self.candidates        = self.words[:]
        self.available_letters = []
        self.block_positions   = {}        # letter -> {x_m, y_m, z_m, theta_deg}
        self.current_guess     = None
        self.attempt           = 1
        self.game_active       = False
        self.last_feedback     = None
        self.guess_pending     = True      # True = ready to pick and publish next guess

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(String, '/perception/detections', self.detections_callback, 10)
        self.create_subscription(String, '/mission/state',          self.mission_callback,    10)
        self.create_subscription(String, '/gamification/feedback',  self.feedback_callback,   10)

        # ── Publishers ────────────────────────────────────────────
        self.pub_guess       = self.create_publisher(String, '/gamification/guess',        10)
        self.pub_mission     = self.create_publisher(String, '/gamification/mission_state', 10)
        self.pub_diagnostics = self.create_publisher(String, '/diagnostics',               10)

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, '/gamification/reset', self.reset_callback)
        self.create_service(Trigger, '/gamification/undo',  self.undo_callback)

        self.get_logger().info(
            '\nGamification node ready.'
            '\n  Subscribing: /perception/detections  /mission/state  /gamification/feedback'
            '\n  Publishing:  /gamification/guess  /gamification/mission_state  /diagnostics'
        )


    # ─────────────────────────────────────────────────────────────
    #  Subscriber Callbacks
    # ─────────────────────────────────────────────────────────────

    def detections_callback(self, msg):
        """
        Receives detected letter blocks from SS1.
        Always updates available letters and positions.
        Only picks and publishes a new guess when guess_pending is True,
        preventing re-guessing on every detection frame.
        """
        if not self.game_active:
            return

        try:
            data   = json.loads(msg.data)
            blocks = data.get('blocks', [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detections JSON: {e}')
            return

        if not blocks:
            return

        # Always refresh letter positions from latest frame
        self.available_letters = []
        self.block_positions   = {}
        for block in blocks:
            letter = block.get('letter', '').upper()
            if letter and letter.isalpha():
                self.available_letters.append(letter)
                self.block_positions[letter] = {
                    'x_m'      : block.get('x_m'),
                    'y_m'      : block.get('y_m'),
                    'z_m'      : block.get('z_m'),
                    'theta_deg': block.get('theta_deg', 0),
                }

        # Only pick a new guess when we're ready for one
        if not self.guess_pending:
            return

        self.get_logger().info(f'Letters available: {self.available_letters}')

        letter_pool = [l.lower() for l in self.available_letters]
        formable    = [
            w for w in self.candidates
            if all(letter_pool.count(c) >= w.count(c) for c in set(w))
        ]

        if not formable:
            self.get_logger().warn('No formable words from detected letters — waiting for more.')
            self._publish_diagnostics(status='WAITING_FOR_LETTERS')
            return

        if self.attempt == 1:
            self.current_guess = choose_opening_guess(formable)
        else:
            self.current_guess = choose_best_guess(formable)

        if not self.current_guess:
            return

        self.get_logger().info(f'Attempt {self.attempt}: Guessing {self.current_guess.upper()}')

        self._publish_guess(self.current_guess)
        self._publish_mission(self.current_guess)
        self._publish_diagnostics(status='GUESSING')

        self.guess_pending = False   # wait for feedback before next guess


    def mission_callback(self, msg):
        """START begins the game, STOP pauses it, RESET clears everything."""
        state = msg.data.upper().strip()

        if state == 'START' and not self.game_active:
            self.game_active  = True
            self.guess_pending = True
            self.get_logger().info('[Mission] Game started.')
            self._publish_diagnostics(status='ACTIVE')

        elif state == 'STOP':
            self.game_active = False
            self.get_logger().info('[Mission] Game paused.')
            self._publish_diagnostics(status='PAUSED')

        elif state == 'RESET':
            self._reset_game()
            self.get_logger().info('[Mission] Game reset.')


    def feedback_callback(self, msg):
        """
        Receives G/B/I feedback from SS3 GUI or voice_node after the robot places letters.
        Filters candidates and sets guess_pending so the next detection triggers a new guess.
        """
        if not self.game_active or not self.current_guess:
            return

        try:
            feedback = parse_feedback(msg.data)
        except ValueError as e:
            self.get_logger().error(f'Invalid feedback: {e}')
            return

        self.get_logger().info(f'Feedback received: {msg.data}  →  {feedback}')

        if all(f == GOOD for f in feedback):
            self.get_logger().info(
                f'SOLVED in {self.attempt} attempt(s)! Word: {self.current_guess.upper()}')
            self._publish_diagnostics(status='SOLVED')
            self.game_active  = False
            self.guess_pending = False
            return

        self.candidates = filter_candidates(self.candidates, self.current_guess, feedback)
        self.get_logger().info(f'{len(self.candidates)} candidates remaining.')

        if not self.candidates:
            self.get_logger().error('No candidates left — feedback may be inconsistent.')
            self._publish_diagnostics(status='ERROR')
            return

        self.attempt      += 1
        self.last_feedback = feedback
        self.guess_pending = True    # ready for next guess on next detection

        self._publish_diagnostics(status='NEXT_TURN')


    # ─────────────────────────────────────────────────────────────
    #  Service Callbacks
    # ─────────────────────────────────────────────────────────────

    def reset_callback(self, request, response):
        self._reset_game()
        response.success = True
        response.message = 'Game reset successfully.'
        self.get_logger().info('[Service] Game reset.')
        return response


    def undo_callback(self, request, response):
        """One-step undo — resets candidates to full list and decrements attempt."""
        if self.attempt > 1:
            self.attempt      -= 1
            self.candidates    = self.words[:]
            self.guess_pending = True
            response.success   = True
            response.message   = f'Undone to attempt {self.attempt}.'
            self.get_logger().info(f'[Service] Undo → attempt {self.attempt}')
        else:
            response.success = False
            response.message = 'Nothing to undo.'
        return response


    # ─────────────────────────────────────────────────────────────
    #  Publishers
    # ─────────────────────────────────────────────────────────────

    def _publish_guess(self, guess):
        msg      = String()
        msg.data = guess.upper()
        self.pub_guess.publish(msg)
        self.get_logger().info(f'Published guess: {guess.upper()}')


    def _publish_mission(self, guess):
        """
        Publishes ordered letter placement sequence to SS2: Motion Planning.

        Format:
        {
          "word": "CRANE",
          "attempt": 1,
          "placements": [
            {"position": 1, "letter": "C", "x_m": 0.04, "y_m": -0.02, "z_m": 0.38, "theta_deg": 0.0},
            ...
          ]
        }
        """
        placements = []
        for i, letter in enumerate(guess.upper()):
            pos = self.block_positions.get(letter, {})
            placements.append({
                'position' : i + 1,
                'letter'   : letter,
                'x_m'      : pos.get('x_m'),
                'y_m'      : pos.get('y_m'),
                'z_m'      : pos.get('z_m'),
                'theta_deg': pos.get('theta_deg', 0.0),
            })

        payload      = json.dumps({'word': guess.upper(), 'attempt': self.attempt, 'placements': placements})
        msg          = String()
        msg.data     = payload
        self.pub_mission.publish(msg)
        self.get_logger().info(f'Published mission state: {payload}')


    def _publish_diagnostics(self, status='ACTIVE'):
        """
        Publishes board state to SS3 GUI.

        Format:
        {
          "status": "ACTIVE",
          "attempt": 2,
          "current_guess": "CRANE",
          "candidates_left": 84,
          "available_letters": ["C","R","A","N","E"],
          "top_candidates": ["CRANE","STARE","TRACE"]
        }
        """
        top     = self.candidates[:5] if self.candidates else []
        payload = json.dumps({
            'status'           : status,
            'attempt'          : self.attempt,
            'current_guess'    : self.current_guess.upper() if self.current_guess else None,
            'candidates_left'  : len(self.candidates),
            'available_letters': self.available_letters,
            'top_candidates'   : [w.upper() for w in top],
        })
        msg      = String()
        msg.data = payload
        self.pub_diagnostics.publish(msg)
        self.get_logger().info(f'Published diagnostics: {status}')


    # ─────────────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────────────

    def _reset_game(self):
        self.candidates        = self.words[:]
        self.available_letters = []
        self.block_positions   = {}
        self.current_guess     = None
        self.attempt           = 1
        self.game_active       = False
        self.last_feedback     = None
        self.guess_pending     = True
        self._publish_diagnostics(status='RESET')


# ─────────────────────────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GamificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
