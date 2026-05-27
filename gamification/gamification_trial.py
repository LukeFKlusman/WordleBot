#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  gamification_trial.py
#
#  TRIAL VERSION of the Subsystem 4 Gamification node.
#
#  Difference from gamification_node.py
#  ─────────────────────────────────────
#  In the original node, Mode A picks a guess from whatever letters
#  perception is *currently* showing. That couples guessing to the
#  detector and makes the robot's choice depend on which blocks
#  happen to be in frame.
#
#  This trial version inverts the flow:
#
#    1. Mode A picks a 5-letter guess from the full candidate list
#       *first*, without looking at the detection stream.
#    2. It then watches /perception/detections and accumulates which
#       letters of that guess it has seen so far (FOUND) vs which it
#       is still waiting on (LOOKING_FOR). Multiplicity is respected:
#       for APPLE the node needs TWO P blocks, not one.
#    3. Once every required letter has been seen at least the right
#       number of times, the node logs "WORD GUESS CONFIRMED" and
#       only then publishes the mission state and HL word request.
#
#  This means the diagnostics stream is much richer mid-search — the
#  SS3 GUI can show "Looking for: P, P, E" in real time while the
#  user arranges blocks on the table.
#
#  TOPICS, SERVICES, QoS:
#    Same as gamification_node.py. See that file's header for the
#    full list. The new diagnostics payload adds three fields:
#      - target_word        the guess we are currently hunting letters for
#      - found_letters      list of letters confirmed so far (with repeats)
#      - looking_for        list of letters still needed (with repeats)
#
#  HOW TO RUN:
#    Same as gamification_node.py — drop-in replacement, just run
#    this file instead of the original.
# ─────────────────────────────────────────────────────────────────

import os
import sys
import json
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
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


class GamificationTrialNode(Node):

    def __init__(self):
        super().__init__('gamification_trial_node')

        dict_path  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
        self.words = load_dictionary(dict_path)
        self.get_logger().info(f'Dictionary loaded: {len(self.words)} words')

        # ── Game state ────────────────────────────────────────────
        self.candidates          = self.words[:]
        self.available_letters   = []       # all letters currently visible to perception
        self.block_positions     = {}       # letter -> {x_m, y_m, z_m, theta_deg}
        self.current_guess       = None
        self.selected_secret     = None
        self.attempt             = 1
        self.game_active         = False
        self.last_feedback       = None
        self.game_mode           = None
        self.mode_locked         = False
        self.last_error          = None
        self.last_scored_attempt = None

        # ── Trial-specific state (Mode A letter hunt) ─────────────
        # required_counter:  Counter of letters the current guess needs
        # found_counter:     Counter of letters detected so far towards that guess
        # guess_confirmed:   True once found_counter satisfies required_counter
        # awaiting_guess:    True when we are between turns and need to pick a new word
        self.required_counter = Counter()
        self.found_counter    = Counter()
        self.guess_confirmed  = False
        self.awaiting_guess   = True

        # ── Latched QoS for HL control ────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(String, '/perception/detections',     self.detections_callback,    10)
        self.create_subscription(String, '/mission/state',             self.mission_callback,        10)
        self.create_subscription(String, '/gamification/feedback',     self.feedback_callback,       10)
        self.create_subscription(String, '/gamification/mode',         self.mode_callback,           10)
        self.create_subscription(String, '/gamification/secret_word',  self.secret_callback,         10)
        self.create_subscription(String, 'hl_control/word_request', self.player_guess_callback,   10)

        # ── Publishers ────────────────────────────────────────────
        self.pub_guess        = self.create_publisher(String, '/gamification/guess',         10)
        self.pub_mission      = self.create_publisher(String, '/gamification/mission_state', 10)
        self.pub_diagnostics  = self.create_publisher(String, '/diagnostics',                10)
        self.pub_word_request = self.create_publisher(
            String, '/hl_control/word_request', latched_qos)

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, '/gamification/reset', self.reset_callback)
        self.create_service(Trigger, '/gamification/undo',  self.undo_callback)

        self.get_logger().info(
            '\nGamification TRIAL node ready.'
            '\n  Flow: pick guess first → wait for all its letters on the table → confirm.'
            '\n  Subscribing: /perception/detections  /mission/state  /gamification/feedback'
            '\n               /gamification/mode  /gamification/secret_word  /gamification/player_guess'
            '\n  Publishing:  /gamification/guess  /gamification/mission_state  /diagnostics'
            '\n               /hl_control/word_request  (latched)'
        )


    # ─────────────────────────────────────────────────────────────
    #  Subscriber Callbacks
    # ─────────────────────────────────────────────────────────────

    def detections_callback(self, msg):
        """
        Always refreshes the visible-letter view from the latest frame.

        In Mode A, if we have a current_guess we are hunting for, we
        check whether the visible letters now cover every required
        slot (with multiplicity). When they do, we confirm the guess
        and dispatch the mission.
        """
        if self.game_mode != 'A' or not self.mode_locked or not self.game_active:
            return

        try:
            data   = json.loads(msg.data)
            blocks = data.get('blocks', [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detections JSON: {e}')
            return

        # Refresh the live view of what perception sees right now.
        # This happens whether or not we have a guess yet — useful for the GUI.
        self.available_letters = []
        self.block_positions   = {}
        for block in blocks:
            letter = block.get('letter', '').upper()
            if letter and letter.isalpha():
                self.available_letters.append(letter)
                # Latest detection wins for positional data.
                self.block_positions[letter] = {
                    'x_m'      : block.get('x_m'),
                    'y_m'      : block.get('y_m'),
                    'z_m'      : block.get('z_m'),
                    'theta_deg': block.get('theta_deg', 0),
                }

        # If we are between turns and need to pick a guess, do that now.
        # We pick from the full candidate list — independent of what's visible.
        if self.awaiting_guess:
            self._pick_and_announce_guess()
            return

        # If we have a guess and it isn't confirmed yet, check letter coverage.
        if self.current_guess and not self.guess_confirmed:
            self._update_letter_hunt()


    def mission_callback(self, msg):
        state = msg.data.upper().strip()

        if state in ('START', 'SCANNING', 'RESUME') and not self.game_active:
            if not self.mode_locked or self.game_mode not in ('A', 'B'):
                self.last_error = 'Choose a game mode before starting.'
                self.get_logger().warn('[Mission] Start ignored because no game mode is selected.')
                self._publish_diagnostics(status='SELECT_MODE')
                return

            self.game_active = True
            self.last_error  = None
            if self.game_mode == 'A':
                # Mode A starts by picking a guess immediately — independent of detections.
                # If perception hasn't published yet, we still announce the target word;
                # the letter hunt will progress as soon as the first frame arrives.
                self.awaiting_guess = True
                if self.selected_secret is None:
                    self.get_logger().warn(
                        '[Mission] Mode A started without a selected secret word; '
                        'manual feedback is still accepted.')
                self.get_logger().info('[Mission] Mode A game started.')
                self._pick_and_announce_guess()
            else:
                self.selected_secret = choose_secret_word(self.words)
                self.current_guess   = None
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
        Receives G/B/I feedback from SS3 / voice after the robot has placed
        the letters. Filters candidates and queues the next guess.
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
            self.game_active = False
            self._publish_diagnostics(status='SOLVED')
            return

        self.last_feedback = feedback
        if self.attempt >= MAX_ATTEMPTS:
            self.get_logger().info(
                f'AI failed in {MAX_ATTEMPTS} attempts. Word was '
                f'{self.selected_secret.upper() if self.selected_secret else "<unknown>"}.')
            self.game_active = False
            self._publish_diagnostics(status='GAME_OVER')
            return

        self.candidates = filter_candidates(self.candidates, self.current_guess, feedback)
        self.get_logger().info(f'{len(self.candidates)} candidates remaining.')

        if not self.candidates:
            self.get_logger().error('No candidates left — feedback may be inconsistent.')
            self._publish_diagnostics(status='ERROR')
            return

        # Set up for the next attempt: clear the letter hunt, then pick a new word.
        self.attempt        += 1
        self.awaiting_guess  = True
        self.guess_confirmed = False
        self.found_counter   = Counter()
        self.required_counter = Counter()
        self.current_guess   = None
        self._pick_and_announce_guess()


    def mode_callback(self, msg):
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

        self.game_mode   = requested
        self.mode_locked = True
        self.last_error  = None
        self.get_logger().info(f'Game mode set to Mode {self.game_mode}.')
        if self.game_mode == 'B':
            self._start_mode_b_game()
            return

        self._publish_diagnostics(status='SELECT_SECRET')


    def secret_callback(self, msg):
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
        self.game_active     = True
        self.awaiting_guess  = True
        self.last_error      = None
        self.get_logger().info('Mode A secret word accepted.')
        self._pick_and_announce_guess()


    def player_guess_callback(self, msg):
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

        self.current_guess       = guess
        self.last_feedback       = score_guess_against_target(guess, self.selected_secret)
        self.last_scored_attempt = self.attempt
        self.last_error          = None

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
        if self.attempt > 1:
            self.attempt        -= 1
            self.candidates      = self.words[:]
            self.current_guess   = None
            self.awaiting_guess  = True
            self.guess_confirmed = False
            self.found_counter   = Counter()
            self.required_counter = Counter()
            response.success     = True
            response.message     = f'Undone to attempt {self.attempt}.'
            self.get_logger().info(f'[Service] Undo → attempt {self.attempt}')
        else:
            response.success = False
            response.message = 'Nothing to undo.'
        return response


    # ─────────────────────────────────────────────────────────────
    #  Core trial behaviour: pick first, hunt letters second
    # ─────────────────────────────────────────────────────────────

    def _pick_and_announce_guess(self):
        """
        Pick a guess from the candidate list independently of what's on
        the table, announce it on /gamification/guess, then begin the
        letter hunt against the current detection stream.
        """
        if self.game_mode != 'A' or not self.mode_locked or not self.game_active:
            return

        if not self.candidates:
            self.get_logger().error('No candidates left — cannot pick a guess.')
            self._publish_diagnostics(status='ERROR')
            return

        if self.attempt == 1:
            self.current_guess = choose_opening_guess(self.candidates)
        else:
            self.current_guess = choose_best_guess(self.candidates)

        if not self.current_guess:
            self._publish_diagnostics(status='ERROR')
            return

        # Set up the letter-hunt state for this guess.
        self.required_counter = Counter(self.current_guess.upper())
        self.found_counter    = Counter()
        self.guess_confirmed  = False
        self.awaiting_guess   = False

        self.get_logger().info(
            f'Attempt {self.attempt}: target word is {self.current_guess.upper()}. '
            f'Looking for letters: '
            f'{self._counter_to_letter_list(self.required_counter)}')

        self._publish_guess(self.current_guess)

        # Run an initial hunt pass in case perception has already given us
        # letters before we picked this word.
        self._update_letter_hunt()


    def _update_letter_hunt(self):
        """
        Compare the latest visible letters against the required multiset
        for the current guess. Update found_counter and, if every slot is
        filled, confirm the guess and dispatch the mission.
        """
        # Cap the found count for each letter at what the word actually needs,
        # so we don't over-count duplicate sightings of a single block across
        # frames. e.g. APPLE requires P:2 — seeing one P twice is still just 1 P.
        visible_counter = Counter(self.available_letters)
        new_found = Counter()
        for letter, needed in self.required_counter.items():
            new_found[letter] = min(visible_counter.get(letter, 0), needed)

        # Log only when something changed, to avoid log spam every frame.
        if new_found != self.found_counter:
            self.found_counter = new_found
            still_needed = self.required_counter - self.found_counter
            self.get_logger().info(
                f'  FOUND: {self._counter_to_letter_list(self.found_counter)}   '
                f'LOOKING FOR: {self._counter_to_letter_list(still_needed)}'
            )
            self._publish_diagnostics(status='HUNTING_LETTERS')

        # Check completion: every required slot satisfied.
        if self.found_counter == self.required_counter:
            self._confirm_guess()


    def _confirm_guess(self):
        """Letters are all on the table — lock in the guess and dispatch."""
        if self.guess_confirmed:
            return
        self.guess_confirmed = True
        self.get_logger().info(
            f'★ WORD GUESS CONFIRMED: {self.current_guess.upper()} — '
            f'all required letters visible.')
        self._publish_mission(self.current_guess)
        self._publish_word_request(self.current_guess)
        self._publish_diagnostics(status='CONFIRMED')


    # ─────────────────────────────────────────────────────────────
    #  Publishers
    # ─────────────────────────────────────────────────────────────

    def _publish_guess(self, guess):
        msg      = String()
        msg.data = guess.upper()
        self.pub_guess.publish(msg)
        self.get_logger().info(f'Published guess: {guess.upper()}')


    def _publish_word_request(self, guess):
        msg      = String()
        msg.data = guess.upper()
        self.pub_word_request.publish(msg)
        self.get_logger().info(f'Published word request to HL control: {guess.upper()}')


    def _publish_mission(self, guess):
        """
        Publishes ordered letter placement sequence to SS2.

        Because the same letter can appear more than once in a word, we
        consume block_positions per-letter as we walk the word so the
        nth occurrence of a letter still gets a position assigned. (All
        copies of a duplicate letter share the same stored position, since
        perception keys by letter — SS2 is responsible for selecting which
        physical block to pick when there are duplicates.)
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

        payload  = json.dumps({
            'word'      : guess.upper(),
            'attempt'   : self.attempt,
            'placements': placements,
        })
        msg      = String()
        msg.data = payload
        self.pub_mission.publish(msg)
        self.get_logger().info(f'Published mission state: {payload}')


    def _publish_diagnostics(self, status='ACTIVE'):
        """
        Publishes board state to SS3 GUI. Adds target_word / found_letters /
        looking_for so the GUI can render the letter-hunt progress.
        """
        top          = self.candidates[:5] if self.candidates else []
        still_needed = self.required_counter - self.found_counter
        payload = json.dumps({
            'status'           : status,
            'mode'             : self.game_mode,
            'mode_locked'      : self.mode_locked,
            'attempt'          : self.attempt,
            'current_guess'    : self.current_guess.upper() if self.current_guess else None,
            'target_word'      : self.current_guess.upper() if self.current_guess else None,
            'found_letters'    : self._counter_to_letter_list(self.found_counter),
            'looking_for'      : self._counter_to_letter_list(still_needed),
            'guess_confirmed'  : self.guess_confirmed,
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
        self.candidates          = self.words[:]
        self.selected_secret     = choose_secret_word(self.words)
        self.current_guess       = None
        self.attempt             = 1
        self.game_active         = True
        self.last_feedback       = None
        self.last_scored_attempt = None
        self.awaiting_guess      = False
        self.guess_confirmed     = False
        self.required_counter    = Counter()
        self.found_counter       = Counter()
        self.last_error          = None
        self.get_logger().info('Mode B game started with hidden solution selected.')
        self._publish_diagnostics(status='ACTIVE')


    def _counter_to_letter_list(self, counter):
        """Counter({'P': 2, 'E': 1}) → ['P', 'P', 'E']  (stable, sorted)."""
        out = []
        for letter in sorted(counter):
            out.extend([letter] * counter[letter])
        return out


    def _feedback_to_chars(self, feedback):
        if not feedback:
            return []
        mapping = {GOOD: 'G', BAD_POSITION: 'B', INCORRECT: 'I'}
        return [mapping.get(token, '') for token in feedback]


    def _reset_game(self, preserve_mode=False):
        mode = self.game_mode
        self.candidates          = self.words[:]
        self.available_letters   = []
        self.block_positions     = {}
        self.current_guess       = None
        self.selected_secret     = None
        self.attempt             = 1
        self.game_active         = False
        self.last_feedback       = None
        self.last_scored_attempt = None
        self.awaiting_guess      = True
        self.guess_confirmed     = False
        self.required_counter    = Counter()
        self.found_counter       = Counter()
        self.last_error          = None
        if preserve_mode:
            self.game_mode   = mode
            self.mode_locked = mode in ('A', 'B')
        else:
            self.game_mode   = None
            self.mode_locked = False
        self._publish_diagnostics(status='RESET')


# ─────────────────────────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GamificationTrialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
