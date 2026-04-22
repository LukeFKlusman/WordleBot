#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  gamification_node.py
#
#  ROS2 node for Subsystem 4 — Advanced Capabilities.
#  Wraps the existing wordle_logic.py in a proper ROS2 node.
#
#  TOPICS SUBSCRIBED:
#    /perception/detections   std_msgs/String  — JSON block list from SS1
#    /mission/state           std_msgs/String  — game start/stop from SS3
#    /gamification/feedback   std_msgs/String  — G/B/I feedback from SS3 GUI
#
#  TOPICS PUBLISHED:
#    /gamification/guess      std_msgs/String  — current word guess
#    /gamification/mission_state std_msgs/String — ordered placement JSON for SS2
#    /diagnostics             std_msgs/String  — board state JSON for SS3 GUI
#
#  SERVICES:
#    /gamification/reset      — resets the game state
#    /gamification/undo       — undoes the last turn
#
#  HOW TO RUN:
#    Terminal 1: ros2 launch realsense2_camera rs_launch.py ...
#    Terminal 2: python3 perception/src/realsense_camera_cnn.py
#    Terminal 3: python3 gamification/gamification_node.py
# ─────────────────────────────────────────────────────────────────

import os
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ── Add gamification folder to path ──────────────────────────────
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dictionary   import load_dictionary
from wordle_logic import (
    filter_candidates,
    choose_opening_guess,
    choose_best_guess,
    parse_feedback,
)
from constants import GOOD, BAD_POSITION, INCORRECT


# ─────────────────────────────────────────────────────────────────
#  Gamification Node
# ─────────────────────────────────────────────────────────────────

class GamificationNode(Node):

    def __init__(self):
        super().__init__('gamification_node')

        # ── Load dictionary ───────────────────────────────────────
        dict_path   = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
        self.words  = load_dictionary(dict_path)
        self.get_logger().info(f'Dictionary loaded: {len(self.words)} words')

        # ── Game state ────────────────────────────────────────────
        self.candidates      = self.words[:]   # all valid words to start
        self.available_letters = []            # letters detected by SS1
        self.current_guess   = None            # current word guess
        self.attempt         = 1              # turn counter
        self.game_active     = False          # only run logic when active
        self.last_feedback   = None           # last feedback received from SS3

        # ── Subscribers ───────────────────────────────────────────

        # From SS1: Perception — detected letter blocks as JSON
        self.create_subscription(
            String,
            '/perception/detections',
            self.detections_callback,
            10
        )

        # From SS3: Interaction and Execution — game start/stop
        self.create_subscription(
            String,
            '/mission/state',
            self.mission_callback,
            10
        )

        # From SS3: GUI feedback after each guess (G/B/I string)
        self.create_subscription(
            String,
            '/gamification/feedback',
            self.feedback_callback,
            10
        )

        # ── Publishers ────────────────────────────────────────────

        # Current guess — SS3 GUI displays this
        self.pub_guess = self.create_publisher(
            String, '/gamification/guess', 10)

        # Ordered placement sequence — SS2 Motion Planning uses this
        # Format: {"word": "CRANE", "placements": [{"letter":"C","x_m":0.04,"y_m":-0.02,"z_m":0.38,"theta_deg":0}, ...]}
        self.pub_mission = self.create_publisher(
            String, '/gamification/mission_state', 10)

        # Board state — SS3 GUI uses this to update display
        self.pub_diagnostics = self.create_publisher(
            String, '/diagnostics', 10)

        # ── Services ─────────────────────────────────────────────

        # Reset game
        self.create_service(
            Trigger,
            '/gamification/reset',
            self.reset_callback
        )

        # Undo last turn
        self.create_service(
            Trigger,
            '/gamification/undo',
            self.undo_callback
        )

        self.get_logger().info(
            '\nGamification node ready.'
            '\n  Subscribing: /perception/detections'
            '\n               /mission/state'
            '\n               /gamification/feedback'
            '\n  Publishing:  /gamification/guess'
            '\n               /gamification/mission_state'
            '\n               /diagnostics'
        )


    # ─────────────────────────────────────────────────────────────
    #  Subscriber Callbacks
    # ─────────────────────────────────────────────────────────────

    def detections_callback(self, msg):
        """
        Receives detected letter blocks from SS1: Perception.
        Parses the JSON, extracts letters, filters candidates,
        then makes a guess and publishes it.

        Only runs when game is active.
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
            self.get_logger().warn('No blocks detected — waiting for letters.')
            return

        # Extract letters and their 3D positions
        self.available_letters = []
        self.block_positions   = {}  # letter -> {x_m, y_m, z_m, theta_deg}

        for block in blocks:
            letter = block.get('letter')
            conf   = block.get('conf', 0)

            if letter and letter.isalpha():
                self.available_letters.append(letter.upper())
                self.block_positions[letter.upper()] = {
                    'x_m'      : block.get('x_m'),
                    'y_m'      : block.get('y_m'),
                    'z_m'      : block.get('z_m'),
                    'theta_deg': block.get('theta_deg', 0),
                }

        self.get_logger().info(
            f'Letters available: {self.available_letters}')

        # Filter candidates to only words formable from available letters
        letter_pool = [l.lower() for l in self.available_letters]
        formable    = [
            w for w in self.candidates
            if all(letter_pool.count(c) >= w.count(c) for c in set(w))
        ]

        if not formable:
            self.get_logger().warn(
                'No valid words formable from available letters. '
                'Waiting for more letters from perception.')
            self._publish_diagnostics(status='WAITING_FOR_LETTERS')
            return

        # Choose best guess from formable candidates
        if self.attempt == 1:
            self.current_guess = choose_opening_guess(formable)
        else:
            self.current_guess = choose_best_guess(formable)

        if not self.current_guess:
            self.get_logger().warn('Could not determine a guess.')
            return

        self.get_logger().info(
            f'Attempt {self.attempt}: Guessing {self.current_guess.upper()}')

        # Publish the guess
        self._publish_guess(self.current_guess)

        # Publish placement sequence to SS2
        self._publish_mission(self.current_guess)

        # Publish board state to SS3
        self._publish_diagnostics(status='GUESSING')


    def mission_callback(self, msg):
        """
        Receives game start/stop from SS3: Interaction and Execution.
        'START'  — begins a new game
        'STOP'   — pauses the game
        'RESET'  — resets everything
        """
        state = msg.data.upper().strip()

        if state == 'START' and not self.game_active:
            self.game_active = True
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
        Receives G/B/I feedback from SS3: Interaction and Execution GUI
        after the robot places letters and the human evaluates the guess.

        Filters candidate list based on feedback and prepares for next turn.
        """
        if not self.game_active or not self.current_guess:
            return

        try:
            feedback = parse_feedback(msg.data)
        except ValueError as e:
            self.get_logger().error(f'Invalid feedback: {e}')
            return

        self.get_logger().info(
            f'Feedback received: {msg.data}  →  {feedback}')

        # Check if solved
        if all(f == GOOD for f in feedback):
            self.get_logger().info(
                f'SOLVED in {self.attempt} attempt(s)! '
                f'Word: {self.current_guess.upper()}')
            self._publish_diagnostics(status='SOLVED')
            self.game_active = False
            return

        # Filter candidates based on feedback
        self.candidates = filter_candidates(
            self.candidates, self.current_guess, feedback)

        self.get_logger().info(
            f'{len(self.candidates)} candidates remaining.')

        self.attempt      += 1
        self.last_feedback = feedback

        # Check if out of candidates
        if not self.candidates:
            self.get_logger().error(
                'No candidates left — feedback may be inconsistent.')
            self._publish_diagnostics(status='ERROR')
            return

        self._publish_diagnostics(status='NEXT_TURN')


    # ─────────────────────────────────────────────────────────────
    #  Service Callbacks
    # ─────────────────────────────────────────────────────────────

    def reset_callback(self, request, response):
        """Resets the full game state."""
        self._reset_game()
        response.success = True
        response.message = 'Game reset successfully.'
        self.get_logger().info('[Service] Game reset.')
        return response


    def undo_callback(self, request, response):
        """
        Undo last turn — resets candidates to full word list
        and decrements attempt counter.
        Note: Full undo history would require storing state per turn.
        This is a simple one-step undo.
        """
        if self.attempt > 1:
            self.attempt    -= 1
            self.candidates  = self.words[:]
            response.success = True
            response.message = f'Undone to attempt {self.attempt}.'
            self.get_logger().info(f'[Service] Undo → attempt {self.attempt}')
        else:
            response.success = False
            response.message = 'Nothing to undo.'
        return response


    # ─────────────────────────────────────────────────────────────
    #  Publishers
    # ─────────────────────────────────────────────────────────────

    def _publish_guess(self, guess):
        """Publishes current guess string to SS3 GUI."""
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
            {"position": 2, "letter": "R", "x_m": 0.11, "y_m": -0.02, "z_m": 0.38, "theta_deg": 5.2},
            ...
          ]
        }

        Each letter's x_m, y_m, z_m comes directly from SS1 perception data.
        SS2 applies the EE-to-robot transform from there.
        """
        placements = []
        for i, letter in enumerate(guess.upper()):
            pos_data = self.block_positions.get(letter, {})
            placements.append({
                'position' : i + 1,
                'letter'   : letter,
                'x_m'      : pos_data.get('x_m'),
                'y_m'      : pos_data.get('y_m'),
                'z_m'      : pos_data.get('z_m'),
                'theta_deg': pos_data.get('theta_deg', 0.0),
            })

        payload      = json.dumps({
            'word'      : guess.upper(),
            'attempt'   : self.attempt,
            'placements': placements,
        })
        msg          = String()
        msg.data     = payload
        self.pub_mission.publish(msg)
        self.get_logger().info(f'Published mission state: {payload}')


    def _publish_diagnostics(self, status='ACTIVE'):
        """
        Publishes board state and game info to SS3: Interaction and Execution.

        Format:
        {
          "status":            "ACTIVE",
          "attempt":           2,
          "current_guess":     "CRANE",
          "candidates_left":   84,
          "available_letters": ["C","R","A","N","E","S","T"],
          "top_candidates":    ["crane","stare","trace"]
        }
        """
        top = self.candidates[:5] if self.candidates else []
        payload  = json.dumps({
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
        """Resets all game state back to initial values."""
        self.candidates        = self.words[:]
        self.available_letters = []
        self.block_positions   = {}
        self.current_guess     = None
        self.attempt           = 1
        self.game_active       = False
        self.last_feedback     = None
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
    