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
#    /gamification/mode          std_msgs/String  — MODE_A or MODE_B
#    /gamification/secret_word   std_msgs/String  — human-selected secret for Mode A
#    /gamification/player_guess  std_msgs/String  — human guess for Mode B
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
    choose_secret_word,
    filter_candidates,
    choose_opening_guess,
    choose_best_guess,
    parse_feedback,
    score_guess_against_target,
)
from constants import GOOD, BAD_POSITION, INCORRECT

MAX_ATTEMPTS = 6


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
        self.selected_secret   = None
        self.attempt           = 1
        self.game_active       = False
        self.last_feedback     = None
        self.guess_pending     = True      # True = ready to pick and publish next guess
        self.game_mode         = None      # A = robot guesses, B = human guesses
        self.mode_locked       = False
        self.last_error        = None
        self.last_scored_attempt = None

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(String, '/perception/detections', self.detections_callback, 10)
        self.create_subscription(String, '/mission/state',          self.mission_callback,    10)
        self.create_subscription(String, '/gamification/feedback',  self.feedback_callback,   10)
        self.create_subscription(String, '/gamification/mode',       self.mode_callback,       10)
        self.create_subscription(String, '/gamification/secret_word', self.secret_callback,    10)
        self.create_subscription(String, '/gamification/player_guess', self.player_guess_callback, 10)

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
            '\n               /gamification/mode  /gamification/secret_word  /gamification/player_guess'
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
        if self.game_mode != 'A' or not self.mode_locked:
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

        if state in ('START', 'SCANNING', 'RESUME') and not self.game_active:
            if not self.mode_locked or self.game_mode not in ('A', 'B'):
                self.last_error = 'Choose a game mode before starting.'
                self.get_logger().warn('[Mission] Start ignored because no game mode is selected.')
                self._publish_diagnostics(status='SELECT_MODE')
                return

            self.game_active  = True
            self.last_error = None
            if self.game_mode == 'A':
                self.guess_pending = True
                if self.selected_secret is None:
                    self.get_logger().warn(
                        '[Mission] Mode A started without a selected secret word; manual feedback is still accepted.')
                self.get_logger().info('[Mission] Mode A game started.')
            else:
                self.selected_secret = choose_secret_word(self.words)
                self.current_guess = None
                self.guess_pending = False
                self.get_logger().info('[Mission] Mode B game started with hidden solution selected.')
            self._publish_diagnostics(status='ACTIVE')

        elif state in ('STOP', 'IDLE'):
            self.game_active = False
            self.get_logger().info('[Mission] Game paused.')
            self._publish_diagnostics(status='PAUSED')

        elif state == 'RESET':
            self._reset_game(preserve_mode=False)
            self.get_logger().info('[Mission] Game reset.')


    def feedback_callback(self, msg):
        """
        Receives G/B/I feedback from SS3 GUI or voice_node after the robot places letters.
        Filters candidates and sets guess_pending so the next detection triggers a new guess.
        """
        if self.game_mode != 'A' or not self.mode_locked:
            self.get_logger().warn('Ignoring manual feedback because Mode A is not active.')
            return

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
            self.game_active  = False
            self.guess_pending = False
            self._publish_diagnostics(status='SOLVED')
            return

        self.last_feedback = feedback
        if self.attempt >= MAX_ATTEMPTS:
            self.get_logger().info(
                f'AI failed in {MAX_ATTEMPTS} attempts. Word was {self.selected_secret.upper()}')
            self.game_active = False
            self.guess_pending = False
            self._publish_diagnostics(status='GAME_OVER')
            return

        self.candidates = filter_candidates(self.candidates, self.current_guess, feedback)
        self.get_logger().info(f'{len(self.candidates)} candidates remaining.')

        if not self.candidates:
            self.get_logger().error('No candidates left — feedback may be inconsistent.')
            self._publish_diagnostics(status='ERROR')
            return

        self.attempt      += 1
        self.guess_pending = True
        self._publish_next_mode_a_guess()


    def mode_callback(self, msg):
        """Switches between Mode A (robot guesses) and Mode B (human guesses)."""
        requested = msg.data.upper().strip().replace('MODE_', '')
        if requested not in ('A', 'B'):
            self.get_logger().error(f'Invalid game mode: {msg.data}')
            self.last_error = f'Invalid game mode: {msg.data}'
            self._publish_diagnostics(status='ERROR')
            return

        if self.mode_locked and requested == self.game_mode:
            self._publish_diagnostics(status='MODE_SELECTED')
            return

        if self.mode_locked:
            self.last_error = 'Reset the game before changing modes.'
            self.get_logger().warn('Ignoring mode change because mode is locked until reset.')
            self._publish_diagnostics(status='MODE_LOCKED')
            return

        self.game_mode = requested
        self.mode_locked = True
        self.last_error = None
        self.get_logger().info(f'Game mode set to Mode {self.game_mode}.')
        if self.game_mode == 'B':
            self._start_mode_b_game()
            return

        self._publish_diagnostics(status='SELECT_SECRET')


    def secret_callback(self, msg):
        """Stores the human-selected secret word for Mode A."""
        word = ''.join(ch for ch in msg.data.lower().strip() if ch.isalpha())
        if len(word) != 5 or word not in self.words:
            self.get_logger().error(f'Invalid Mode A secret word: {msg.data}')
            self.last_error = 'Secret word must be a valid five-letter dictionary word.'
            self._publish_diagnostics(status='INVALID_SECRET')
            return

        if self.game_mode != 'A' or not self.mode_locked:
            self.get_logger().warn('Ignoring selected secret because Mode A is not active.')
            return

        self.selected_secret = word
        self.game_active = True
        self.guess_pending = True
        self.last_error = None
        self.get_logger().info('Mode A secret word accepted.')
        self._publish_next_mode_a_guess()


    def player_guess_callback(self, msg):
        """Scores a human guess against the hidden Mode B solution."""
        if self.game_mode != 'B' or not self.mode_locked:
            self.get_logger().warn('Ignoring player guess because Mode B is not active.')
            return

        if not self.game_active or not self.selected_secret:
            self.last_error = 'Mode B is not active.'
            self._publish_diagnostics(status='WAITING_FOR_START')
            return

        guess = ''.join(ch for ch in msg.data.lower().strip() if ch.isalpha())
        if len(guess) != 5 or guess not in self.words:
            self.last_error = 'Guess must be a valid five-letter dictionary word.'
            self.get_logger().error(f'Invalid player guess: {msg.data}')
            self._publish_diagnostics(status='INVALID_GUESS')
            return

        self.current_guess = guess
        self.last_feedback = score_guess_against_target(guess, self.selected_secret)
        self.last_scored_attempt = self.attempt
        self.last_error = None

        if all(f == GOOD for f in self.last_feedback):
            self.get_logger().info(
                f'PLAYER SOLVED in {self.attempt} attempt(s)! Word: {guess.upper()}')
            self.game_active = False
            self._publish_diagnostics(status='SOLVED')
            return

        if self.attempt >= MAX_ATTEMPTS:
            self.get_logger().info(
                f'Mode B game over. Solution was {self.selected_secret.upper()}')
            self.game_active = False
            self._publish_diagnostics(status='GAME_OVER')
            return

        self.attempt += 1
        self._publish_diagnostics(status='NEXT_TURN')


    # ─────────────────────────────────────────────────────────────
    #  Service Callbacks
    # ─────────────────────────────────────────────────────────────

    def reset_callback(self, request, response):
        self._reset_game(preserve_mode=False)
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
            'mode'             : self.game_mode,
            'mode_locked'      : self.mode_locked,
            'attempt'          : self.attempt,
            'current_guess'    : self.current_guess.upper() if self.current_guess else None,
            'candidates_left'  : len(self.candidates),
            'available_letters': self.available_letters,
            'top_candidates'   : [w.upper() for w in top],
            'last_feedback'    : ''.join(self._feedback_to_chars(self.last_feedback)),
            'scored_attempt'   : self.last_scored_attempt,
            'secret_word_set'  : self.selected_secret is not None,
            'solution_word'    : self.selected_secret.upper()
                if status in ('SOLVED', 'GAME_OVER') and self.selected_secret
                else None,
            'game_active'      : self.game_active,
            'max_attempts'     : MAX_ATTEMPTS,
            'error'            : self.last_error,
        })
        msg      = String()
        msg.data = payload
        self.pub_diagnostics.publish(msg)
        self.get_logger().info(f'Published diagnostics: {status}')


    # ─────────────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────────────

    def _start_mode_b_game(self):
        self.candidates = self.words[:]
        self.selected_secret = choose_secret_word(self.words)
        self.current_guess = None
        self.attempt = 1
        self.game_active = True
        self.last_feedback = None
        self.last_scored_attempt = None
        self.guess_pending = False
        self.last_error = None
        self.get_logger().info('Mode B game started with hidden solution selected.')
        self._publish_diagnostics(status='ACTIVE')


    def _publish_next_mode_a_guess(self):
        if self.game_mode != 'A' or not self.mode_locked or not self.game_active:
            return False

        if not self.guess_pending:
            return False

        formable = self.candidates[:]
        if self.available_letters:
            letter_pool = [l.lower() for l in self.available_letters]
            detected_formable = [
                w for w in self.candidates
                if all(letter_pool.count(c) >= w.count(c) for c in set(w))
            ]
            if detected_formable:
                formable = detected_formable
            else:
                self.get_logger().warn(
                    'No candidate can be formed from detected letters; using candidate list for next guess.')

        if not formable:
            self.get_logger().error('No candidates left — feedback may be inconsistent.')
            self._publish_diagnostics(status='ERROR')
            return False

        if self.attempt == 1:
            self.current_guess = choose_opening_guess(formable)
        else:
            self.current_guess = choose_best_guess(formable)

        if not self.current_guess:
            self._publish_diagnostics(status='ERROR')
            return False

        self.get_logger().info(f'Attempt {self.attempt}: Guessing {self.current_guess.upper()}')
        self._publish_guess(self.current_guess)
        self._publish_mission(self.current_guess)
        self._publish_diagnostics(status='GUESSING')
        self.guess_pending = False
        return True

    def _feedback_to_chars(self, feedback):
        if not feedback:
            return []

        mapping = {
            GOOD: 'G',
            BAD_POSITION: 'B',
            INCORRECT: 'I',
        }
        return [mapping.get(token, '') for token in feedback]


    def _reset_game(self, preserve_mode=False):
        mode = self.game_mode
        self.candidates        = self.words[:]
        self.available_letters = []
        self.block_positions   = {}
        self.current_guess     = None
        self.selected_secret   = None
        self.attempt           = 1
        self.game_active       = False
        self.last_feedback     = None
        self.guess_pending     = True
        self.last_error        = None
        self.last_scored_attempt = None
        if preserve_mode:
            self.game_mode = mode
            self.mode_locked = mode in ('A', 'B')
        else:
            self.game_mode = None
            self.mode_locked = False
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
