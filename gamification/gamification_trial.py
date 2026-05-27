#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  gamification_trial.py
#
#  TRIAL VERSION of the Subsystem 4 Gamification node.
#
#  Flow (Mode A):
#  ──────────────
#  1. Pick a 5-letter guess from the candidate list (independent of
#     what perception is currently seeing).
#  2. PRE-MOTION CONFIRMATION
#       - Start a 30 second timer.
#       - Watch /perception/detections. Any letter seen is added to
#         FOUND and STAYS there for the rest of this phase — letters
#         do not have to be visible simultaneously.
#       - If FOUND covers every slot of the guess (with multiplicity)
#         before the timer expires:
#             publish "CONFIRMED" on /gamification/confirmation_status
#             publish mission state + word request to SS2 (Connor)
#       - If the timer expires first:
#             publish "CONFIRMATION_FAILURE"
#             halt the game until /gamification/reset
#  3. SS2 runs motion planning. When it finishes, it publishes
#     FINISHED on /motion_planning/status.
#  4. POST-MOTION CONFIRMATION
#       - Same 30 second sticky-FOUND logic against the same guess.
#       - Success → publish "CONFIRMED" on the same status topic
#         (this time tagged "post_motion"). Game proceeds to wait
#         for G/B/I feedback from SS3.
#       - Timeout → "CONFIRMATION_FAILURE", game halts.
#
#  GUI toggle:
#  ───────────
#  SS3 publishes "true" or "false" on /gamification/confirmation_enabled.
#  When false, both confirmation phases are skipped: as soon as a
#  guess is picked, the mission state is dispatched. SS2's FINISHED
#  signal is also ignored (we go straight to waiting for feedback).
#  Default is enabled.
#
#  Sticky FOUND vs the old flow:
#  ─────────────────────────────
#  The previous version clamped found_counter to whatever letters
#  were visible in the latest frame. That meant a letter that
#  briefly entered frame and then left would not count. This version
#  uses set-union semantics: once a letter is seen, it stays in
#  FOUND for the duration of the confirmation phase. Resets between
#  phases / between attempts.
#
#  NEW TOPICS (not in the original node):
#    /gamification/confirmation_status   String — "CONFIRMED" / "CONFIRMATION_FAILURE"
#    /gamification/confirmation_enabled  String — "true" / "false"   [subscribed]
#    /motion_planning/status             String — "FINISHED"          [subscribed]
#
#  Everything else (subscriptions, publishers, services, QoS) is
#  identical to gamification_node.py — drop-in replacement.
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

MAX_ATTEMPTS         = 6
CONFIRMATION_TIMEOUT = 30.0   # seconds — applies to both pre- and post-motion phases

# Confirmation phase tags
PHASE_IDLE        = 'idle'
PHASE_PRE_MOTION  = 'pre_motion'
PHASE_POST_MOTION = 'post_motion'


class GamificationTrialNode(Node):

    def __init__(self):
        super().__init__('gamification_trial_node')

        dict_path  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
        self.words = load_dictionary(dict_path)
        self.get_logger().info(f'Dictionary loaded: {len(self.words)} words')

        # ── Core game state ───────────────────────────────────────
        self.candidates          = self.words[:]
        self.available_letters   = []
        self.block_positions     = {}
        self.current_guess       = None
        self.selected_secret     = None
        self.attempt             = 1
        self.game_active         = False
        self.last_feedback       = None
        self.game_mode           = None
        self.mode_locked         = False
        self.last_error          = None
        self.last_scored_attempt = None

        # ── Confirmation state ───────────────────────────────────
        # pre_confirmation_enabled  : GUI button #1 — controls the pre-motion scan.
        # post_confirmation_enabled : GUI button #2 — controls the post-motion scan.
        #                             Independent: any combination is valid.
        # confirmation_phase  : which phase we're in (idle / pre / post).
        # required_counter    : letters the current guess needs (multiset).
        # found_counter       : letters seen since this phase began. STICKY:
        #                       once added, never removed within the phase.
        # phase_timer         : 30s timeout, recreated each phase.
        self.pre_confirmation_enabled  = True
        self.post_confirmation_enabled = True
        self.confirmation_phase        = PHASE_IDLE
        self.required_counter          = Counter()
        self.found_counter             = Counter()
        self.phase_timer               = None

        # ── QoS ──────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(String, '/perception/detections',     self.detections_callback,        10)
        self.create_subscription(String, '/mission/state',             self.mission_callback,            10)
        self.create_subscription(String, '/gamification/feedback',     self.feedback_callback,           10)
        self.create_subscription(String, '/gamification/mode',         self.mode_callback,               10)
        self.create_subscription(String, '/gamification/secret_word',  self.secret_callback,             10)
        self.create_subscription(String, '/gamification/player_guess', self.player_guess_callback,       10)
        # New subscriptions for confirmation flow — two independent GUI buttons
        self.create_subscription(String, '/gamification/pre_confirmation_enabled',
                                 self.pre_confirmation_toggle_callback,  10)
        self.create_subscription(String, '/gamification/post_confirmation_enabled',
                                 self.post_confirmation_toggle_callback, 10)
        self.create_subscription(String, '/motion_planning/status',
                                 self.motion_status_callback,            10)

        # ── Publishers ────────────────────────────────────────────
        self.pub_guess         = self.create_publisher(String, '/gamification/guess',               10)
        self.pub_mission       = self.create_publisher(String, '/gamification/mission_state',       10)
        self.pub_diagnostics   = self.create_publisher(String, '/diagnostics',                      10)
        self.pub_confirmation  = self.create_publisher(String, '/gamification/confirmation_status', 10)
        self.pub_word_request  = self.create_publisher(String, '/hl_control/word_request', latched_qos)

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, '/gamification/reset', self.reset_callback)
        self.create_service(Trigger, '/gamification/undo',  self.undo_callback)

        self.get_logger().info(
            '\nGamification TRIAL node ready.'
            '\n  Mode A flow: pick guess → pre-motion confirm (30s) → SS2 motion'
            '\n               → post-motion confirm (30s) → wait for G/B/I feedback'
            '\n  Pre-motion toggle  (GUI button 1): /gamification/pre_confirmation_enabled  (true/false)'
            '\n  Post-motion toggle (GUI button 2): /gamification/post_confirmation_enabled (true/false)'
            '\n  Motion done signal:                /motion_planning/status                  (FINISHED)'
            '\n  Confirmation status:               /gamification/confirmation_status        (CONFIRMED / CONFIRMATION_FAILURE)'
        )


    # ─────────────────────────────────────────────────────────────
    #  Perception callback — drives the letter hunt
    # ─────────────────────────────────────────────────────────────

    def detections_callback(self, msg):
        if self.game_mode not in ('A', 'B') or not self.mode_locked or not self.game_active:
            return

        try:
            data   = json.loads(msg.data)
            blocks = data.get('blocks', [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detections JSON: {e}')
            return

        # Always refresh the live view of visible letters + positions.
        # Latest position wins for each letter (perception keys by letter).
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

        # Only progress the letter hunt while a confirmation phase is active.
        if self.confirmation_phase in (PHASE_PRE_MOTION, PHASE_POST_MOTION):
            self._update_letter_hunt()


    # ─────────────────────────────────────────────────────────────
    #  Mission / mode / feedback callbacks
    # ─────────────────────────────────────────────────────────────

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
                self.get_logger().info('[Mission] Mode A game started.')
                self._start_next_attempt()
            else:
                self.selected_secret = choose_secret_word(self.words)
                self.current_guess   = None
                self.get_logger().info('[Mission] Mode B game started with hidden solution selected.')
                self._publish_diagnostics(status='ACTIVE')

        elif state in ('STOP', 'IDLE'):
            self.game_active = False
            self._cancel_phase_timer()
            self.confirmation_phase = PHASE_IDLE
            self.get_logger().info('[Mission] Game paused.')
            self._publish_diagnostics(status='PAUSED')

        elif state == 'RESET':
            self._reset_game(preserve_mode=False)
            self.get_logger().info('[Mission] Game reset.')


    def feedback_callback(self, msg):
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

        self.attempt += 1
        self._start_next_attempt()


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
        self.last_error      = None
        self.get_logger().info('Mode A secret word accepted.')
        self._start_next_attempt()


    def player_guess_callback(self, msg):
        if self.game_mode != 'B' or not self.mode_locked:
            self.get_logger().warn('Ignoring player guess because Mode B is not active.')
            return
        if not self.game_active or not self.selected_secret:
            self.last_error = 'Mode B is not active.'
            self._publish_diagnostics(status='WAITING_FOR_START')
            return

        # Reject if we're already in the middle of dispatching the previous
        # guess to SS2 — wait for that turn to finish before accepting a new one.
        if self.confirmation_phase in (PHASE_PRE_MOTION, PHASE_POST_MOTION):
            self.last_error = 'Previous guess is still being executed by the robot.'
            self.get_logger().warn('Ignoring player guess — robot is mid-turn.')
            self._publish_diagnostics(status='BUSY')
            return

        guess = ''.join(ch for ch in msg.data.lower().strip() if ch.isalpha())
        if len(guess) != 5 or guess not in self.words:
            self.last_error = 'Guess must be a valid five-letter dictionary word.'
            self.get_logger().error(f'Invalid player guess: {msg.data}')
            self._publish_diagnostics(status='INVALID_GUESS')
            return

        # Score now but DON'T act on the result yet — the robot still needs to
        # place the blocks, and we want the post-motion confirmation to gate
        # the SOLVED / GAME_OVER / NEXT_TURN transition. Scoring is cached on
        # last_feedback so the GUI can show it once the row is placed.
        self.current_guess       = guess
        self.last_feedback       = score_guess_against_target(guess, self.selected_secret)
        self.last_scored_attempt = self.attempt
        self.last_error          = None

        self.get_logger().info(
            f'Mode B attempt {self.attempt}: player guess {guess.upper()}. '
            f'Dispatching to SS2.')
        self._publish_guess(guess)

        # Mirror Mode A: pre-motion confirmation (or skip it) → mission → SS2.
        if not self.pre_confirmation_enabled:
            self.get_logger().info(
                'Pre-motion confirmation disabled — dispatching mission immediately.')
            self.confirmation_phase = PHASE_IDLE
            self.required_counter   = Counter(guess.upper())
            self.found_counter      = Counter()
            self._dispatch_mission()
            return

        self._begin_confirmation_phase(PHASE_PRE_MOTION)


    def _finalise_mode_b_turn(self):
        """
        Called after Mode B's post-motion confirmation succeeds (or after
        SS2 reports FINISHED when post-motion confirmations are disabled).
        Applies the cached feedback to advance the game.
        """
        if self.game_mode != 'B' or not self.last_feedback:
            return

        if all(f == GOOD for f in self.last_feedback):
            self.get_logger().info(
                f'PLAYER SOLVED in {self.attempt} attempt(s)! '
                f'Word: {self.current_guess.upper()}')
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
    #  New callbacks: confirmation toggle + motion-planning signal
    # ─────────────────────────────────────────────────────────────

    def pre_confirmation_toggle_callback(self, msg):
        """GUI button #1 — toggles the pre-motion scan/timeout."""
        new_value = self._parse_bool_token(msg.data)
        if new_value is None:
            self.get_logger().warn(
                f'Ignoring unknown pre_confirmation_enabled value: {msg.data!r}')
            return
        if new_value == self.pre_confirmation_enabled:
            return
        self.pre_confirmation_enabled = new_value
        self.get_logger().info(
            f'Pre-motion confirmation {"ENABLED" if new_value else "DISABLED"} via GUI.')
        # Mid-phase toggles don't cancel a running scan — they take effect
        # on the next guess. Less surprising than killing a scan halfway.
        self._publish_diagnostics(
            status='PRE_CONFIRMATION_ENABLED' if new_value else 'PRE_CONFIRMATION_DISABLED')


    def post_confirmation_toggle_callback(self, msg):
        """GUI button #2 — toggles the post-motion scan/timeout."""
        new_value = self._parse_bool_token(msg.data)
        if new_value is None:
            self.get_logger().warn(
                f'Ignoring unknown post_confirmation_enabled value: {msg.data!r}')
            return
        if new_value == self.post_confirmation_enabled:
            return
        self.post_confirmation_enabled = new_value
        self.get_logger().info(
            f'Post-motion confirmation {"ENABLED" if new_value else "DISABLED"} via GUI.')
        self._publish_diagnostics(
            status='POST_CONFIRMATION_ENABLED' if new_value else 'POST_CONFIRMATION_DISABLED')


    def motion_status_callback(self, msg):
        """
        SS2 (Connor) reports motion-planning status. We act on 'FINISHED'
        (also 'DONE' / 'COMPLETE' for safety). Anything else is logged
        but ignored — failure reporting from SS2 is its own concern.
        """
        if self.game_mode not in ('A', 'B') or not self.mode_locked or not self.game_active:
            return

        token = msg.data.strip().upper()
        if token not in ('FINISHED', 'DONE', 'COMPLETE'):
            self.get_logger().info(f'[Motion] status={token!r} (no action)')
            return

        # If post-motion confirmation is off, skip the scan entirely.
        if not self.post_confirmation_enabled:
            self.get_logger().info(
                '[Motion] FINISHED received. Post-motion confirmation disabled — '
                'skipping post-motion scan.')
            self.confirmation_phase = PHASE_IDLE
            # Mode A: wait for human G/B/I feedback. Mode B: apply cached score.
            if self.game_mode == 'B':
                self._finalise_mode_b_turn()
            else:
                self._publish_diagnostics(status='AWAITING_FEEDBACK')
            return

        if self.confirmation_phase == PHASE_POST_MOTION:
            self.get_logger().warn('[Motion] FINISHED received while already in post-motion phase — ignoring.')
            return

        self.get_logger().info('[Motion] FINISHED received. Starting post-motion confirmation.')
        self._begin_confirmation_phase(PHASE_POST_MOTION)


    # ─────────────────────────────────────────────────────────────
    #  Service callbacks
    # ─────────────────────────────────────────────────────────────

    def reset_callback(self, request, response):
        self._reset_game(preserve_mode=False)
        response.success = True
        response.message = 'Game reset successfully.'
        self.get_logger().info('[Service] Game reset.')
        return response


    def undo_callback(self, request, response):
        if self.attempt > 1:
            self._cancel_phase_timer()
            self.attempt           -= 1
            self.candidates         = self.words[:]
            self.current_guess      = None
            self.confirmation_phase = PHASE_IDLE
            self.required_counter   = Counter()
            self.found_counter      = Counter()
            response.success        = True
            response.message        = f'Undone to attempt {self.attempt}.'
            self.get_logger().info(f'[Service] Undo → attempt {self.attempt}')
        else:
            response.success = False
            response.message = 'Nothing to undo.'
        return response


    # ─────────────────────────────────────────────────────────────
    #  Attempt lifecycle: pick → pre-confirm → motion → post-confirm
    # ─────────────────────────────────────────────────────────────

    def _start_next_attempt(self):
        """Pick a guess and kick off the pre-motion confirmation (or skip it)."""
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

        self.get_logger().info(
            f'Attempt {self.attempt}: target word is {self.current_guess.upper()}.')
        self._publish_guess(self.current_guess)

        if not self.pre_confirmation_enabled:
            # Pre-motion scan disabled: dispatch the mission immediately.
            self.get_logger().info(
                'Pre-motion confirmation disabled — dispatching mission without pre-motion scan.')
            self.confirmation_phase = PHASE_IDLE
            self.required_counter   = Counter(self.current_guess.upper())
            self.found_counter      = Counter()
            self._dispatch_mission()
            return

        self._begin_confirmation_phase(PHASE_PRE_MOTION)


    def _begin_confirmation_phase(self, phase):
        """
        Start a fresh confirmation phase: clear FOUND, set the required
        multiset, arm the 30s timer, and run an initial hunt pass in case
        perception has already delivered some letters.
        """
        self._cancel_phase_timer()
        self.confirmation_phase = phase
        self.required_counter   = Counter(self.current_guess.upper())
        self.found_counter      = Counter()

        label = 'PRE-MOTION' if phase == PHASE_PRE_MOTION else 'POST-MOTION'
        self.get_logger().info(
            f'\n──── {label} CONFIRMATION ────'
            f'\n  Target: {self.current_guess.upper()}'
            f'\n  Required: {self._counter_to_letter_list(self.required_counter)}'
            f'\n  Timeout: {CONFIRMATION_TIMEOUT:.0f}s   (FOUND letters are sticky)'
        )
        self._publish_diagnostics(
            status='CONFIRMING_PRE_MOTION' if phase == PHASE_PRE_MOTION
                   else 'CONFIRMING_POST_MOTION')

        # Arm the timer. One-shot semantics via cancel-on-fire inside the callback.
        self.phase_timer = self.create_timer(CONFIRMATION_TIMEOUT, self._on_phase_timeout)

        # Run one immediate hunt pass against whatever's currently visible.
        self._update_letter_hunt()


    def _update_letter_hunt(self):
        """
        STICKY accumulation: for each required letter, found = max(found, visible).
        Once a letter is in FOUND, it cannot leave during this phase.
        Clamped at required count (no over-collection).
        """
        if self.confirmation_phase not in (PHASE_PRE_MOTION, PHASE_POST_MOTION):
            return
        if not self.current_guess:
            return

        visible_counter = Counter(self.available_letters)
        changed = False
        for letter, needed in self.required_counter.items():
            seen_now      = visible_counter.get(letter, 0)
            already_found = self.found_counter.get(letter, 0)
            new_value     = min(max(already_found, seen_now), needed)
            if new_value != already_found:
                self.found_counter[letter] = new_value
                changed = True

        if changed:
            still_needed = self.required_counter - self.found_counter
            self.get_logger().info(
                f'  FOUND: {self._counter_to_letter_list(self.found_counter)}   '
                f'LOOKING FOR: {self._counter_to_letter_list(still_needed)}'
            )
            self._publish_diagnostics(
                status='HUNTING_PRE_MOTION' if self.confirmation_phase == PHASE_PRE_MOTION
                       else 'HUNTING_POST_MOTION')

        # Completion: every required slot filled.
        if self.found_counter == self.required_counter:
            self._on_phase_success()


    def _on_phase_success(self):
        """All required letters collected before the timer expired."""
        phase = self.confirmation_phase
        self._cancel_phase_timer()
        self.confirmation_phase = PHASE_IDLE

        label = 'PRE-MOTION' if phase == PHASE_PRE_MOTION else 'POST-MOTION'
        self.get_logger().info(
            f'★ {label} CONFIRMED — guess "{self.current_guess.upper()}" '
            f'all letters present.')
        self._publish_confirmation_status('CONFIRMED', phase)

        if phase == PHASE_PRE_MOTION:
            # Hand off to SS2.
            self._dispatch_mission()
            self._publish_diagnostics(status='AWAITING_MOTION_DONE')
        else:
            # Post-motion success.
            # Mode A: wait for the human to enter G/B/I feedback.
            # Mode B: we already scored the guess internally — apply the result.
            if self.game_mode == 'B':
                self._finalise_mode_b_turn()
            else:
                self._publish_diagnostics(status='AWAITING_FEEDBACK')


    def _on_phase_timeout(self):
        """
        30s elapsed without collecting all letters. Halt the game and
        report which letters were missing. User can /gamification/reset
        to try again.
        """
        # rclpy timers fire repeatedly — cancel immediately so this is one-shot.
        self._cancel_phase_timer()
        phase = self.confirmation_phase
        self.confirmation_phase = PHASE_IDLE
        self.game_active = False

        still_needed = self.required_counter - self.found_counter
        label = 'PRE-MOTION' if phase == PHASE_PRE_MOTION else 'POST-MOTION'
        self.last_error = (
            f'{label} confirmation failed. Missing: '
            f'{self._counter_to_letter_list(still_needed)}'
        )
        self.get_logger().error(f'✗ {self.last_error}')
        self._publish_confirmation_status('CONFIRMATION_FAILURE', phase, still_needed)
        self._publish_diagnostics(status='CONFIRMATION_FAILURE')


    def _dispatch_mission(self):
        """Send mission state + latched word request to downstream subsystems."""
        self._publish_mission(self.current_guess)
        self._publish_word_request(self.current_guess)


    def _cancel_phase_timer(self):
        if self.phase_timer is not None:
            self.phase_timer.cancel()
            try:
                self.destroy_timer(self.phase_timer)
            except Exception:
                pass
            self.phase_timer = None


    # ─────────────────────────────────────────────────────────────
    #  Publishers
    # ─────────────────────────────────────────────────────────────

    def _publish_guess(self, guess):
        msg = String(); msg.data = guess.upper()
        self.pub_guess.publish(msg)
        self.get_logger().info(f'Published guess: {guess.upper()}')


    def _publish_word_request(self, guess):
        msg = String(); msg.data = guess.upper()
        self.pub_word_request.publish(msg)
        self.get_logger().info(f'Published word request to HL control: {guess.upper()}')


    def _publish_mission(self, guess):
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

        payload = json.dumps({
            'word'      : guess.upper(),
            'attempt'   : self.attempt,
            'placements': placements,
        })
        msg = String(); msg.data = payload
        self.pub_mission.publish(msg)
        self.get_logger().info(f'Published mission state: {payload}')


    def _publish_confirmation_status(self, status, phase, missing=None):
        """
        status : 'CONFIRMED' or 'CONFIRMATION_FAILURE'
        phase  : 'pre_motion' / 'post_motion'
        missing: optional Counter of letters not found (for failure reports)
        """
        payload = {
            'status' : status,
            'phase'  : phase,
            'word'   : self.current_guess.upper() if self.current_guess else None,
            'attempt': self.attempt,
        }
        if missing is not None:
            payload['missing'] = self._counter_to_letter_list(missing)
        msg = String(); msg.data = json.dumps(payload)
        self.pub_confirmation.publish(msg)
        self.get_logger().info(f'Published confirmation status: {payload}')


    def _publish_diagnostics(self, status='ACTIVE'):
        top          = self.candidates[:5] if self.candidates else []
        still_needed = self.required_counter - self.found_counter
        payload = json.dumps({
            'status'                    : status,
            'mode'                      : self.game_mode,
            'mode_locked'               : self.mode_locked,
            'attempt'                   : self.attempt,
            'current_guess'             : self.current_guess.upper() if self.current_guess else None,
            'target_word'               : self.current_guess.upper() if self.current_guess else None,
            'pre_confirmation_enabled'  : self.pre_confirmation_enabled,
            'post_confirmation_enabled' : self.post_confirmation_enabled,
            'confirmation_phase'        : self.confirmation_phase,
            'found_letters'             : self._counter_to_letter_list(self.found_counter),
            'looking_for'               : self._counter_to_letter_list(still_needed),
            'candidates_left'           : len(self.candidates),
            'available_letters'         : self.available_letters,
            'top_candidates'            : [w.upper() for w in top],
            'last_feedback'             : ''.join(self._feedback_to_chars(self.last_feedback)),
            'scored_attempt'            : self.last_scored_attempt,
            'secret_word_set'           : self.selected_secret is not None,
            'solution_word'             : self.selected_secret.upper()
                if status in ('SOLVED', 'GAME_OVER') and self.selected_secret
                else None,
            'game_active'               : self.game_active,
            'max_attempts'              : MAX_ATTEMPTS,
            'error'                     : self.last_error,
        })
        msg = String(); msg.data = payload
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
        self.confirmation_phase  = PHASE_IDLE
        self.required_counter    = Counter()
        self.found_counter       = Counter()
        self.last_error          = None
        self.get_logger().info('Mode B game started with hidden solution selected.')
        self._publish_diagnostics(status='ACTIVE')


    def _counter_to_letter_list(self, counter):
        """Counter({'P': 2, 'E': 1}) → ['E', 'P', 'P']  (alphabetical, with repeats)."""
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
        self._cancel_phase_timer()
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
        self.confirmation_phase  = PHASE_IDLE
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
#  Entry point
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
