#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  perception_test.py
#
#  ROS2 version — subscribes to Luke's /perception/detections
#  and runs the gamification letter collection logic.
#
#  HOW TO RUN (on the ROS2 laptop in WSL):
#    Terminal 1: Luke runs his perception node
#    Terminal 2: python3 gamification/perception_test.py
#
#  CONTROLS:
#    Enter     — submit G/B/I feedback after confirmed guess
#    show      — show current status
#    reset     — reset letter buffer
#    quit      — quit
# ─────────────────────────────────────────────────────────────────

import os
import sys
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dictionary   import load_dictionary
from wordle_logic import choose_opening_guess, choose_best_guess, filter_candidates, parse_feedback
from constants    import GOOD, BAD_POSITION, INCORRECT


# ─────────────────────────────────────────────────────────────────
#  Game State
# ─────────────────────────────────────────────────────────────────

class GameState:
    def __init__(self, words):
        self.words            = words
        self.candidates       = words[:]
        self.attempt          = 1
        self.current_guess    = None
        self.required_letters = []
        self.letter_buffer    = {}   # letter -> block data from perception
        self.prev_seen        = set()
        self.confirmed        = False
        self.wait_count       = 0
        self.running          = True

    def next_guess(self):
        """Pick next best guess and set required letters."""
        if self.attempt == 1:
            self.current_guess = choose_opening_guess(self.words)
        else:
            self.current_guess = choose_best_guess(self.candidates)

        if not self.current_guess:
            print("\n  No valid guess available.\n")
            return

        self.required_letters = list(self.current_guess.upper())
        self.letter_buffer    = {}
        self.prev_seen        = set()
        self.confirmed        = False
        self.wait_count       = 0

        print(f"\n  {'=' * 50}")
        print(f"  Attempt {self.attempt}")
        print(f"  Target word : {self.current_guess.upper()}")
        print(f"  Place these letters under the camera:")
        print(f"    {' '.join(self.required_letters)}")
        print(f"  {'=' * 50}\n")

    def process_detections(self, blocks):
        """
        Called every time perception publishes a detection frame.
        Stores new letters as they appear — persistent buffer.
        Letters that disappear from frame stay stored.
        """
        # Build current frame letter set
        current_seen = {}
        for block in blocks:
            letter = block.get('letter', '').upper()
            conf   = block.get('conf', 0)
            if letter and letter.isalpha():
                if letter not in current_seen or conf > current_seen[letter].get('conf', 0):
                    current_seen[letter] = {
                        'letter'   : letter,
                        'conf'     : conf,
                        'x_m'      : block.get('x_m'),
                        'y_m'      : block.get('y_m'),
                        'z_m'      : block.get('z_m'),
                        'theta_deg': block.get('theta_deg', 0.0),
                    }

        current_set = set(current_seen.keys())
        new_letters = current_set - self.prev_seen

        for letter in new_letters:
            if letter not in self.letter_buffer:
                self.letter_buffer[letter] = current_seen[letter]
                self.wait_count = 0
                print(f"\n  + New letter detected: {letter}"
                      f"  (conf={current_seen[letter]['conf']:.1f}%"
                      f"  pos=({current_seen[letter].get('x_m')}, "
                      f"{current_seen[letter].get('y_m')}, "
                      f"{current_seen[letter].get('z_m')}))")
            else:
                # Update position with latest reading
                self.letter_buffer[letter] = current_seen[letter]

        self.prev_seen = current_set

        if self.required_letters:
            self.check_match()

    def check_match(self):
        """Check collected letters against required. Flag issues."""
        collected     = list(self.letter_buffer.keys())
        required_copy = self.required_letters[:]

        matched   = []
        missing   = []
        remaining = list(collected)

        for letter in required_copy:
            if letter in remaining:
                matched.append(letter)
                remaining.remove(letter)
            else:
                missing.append(letter)

        extra = remaining

        self._print_live_status(matched, missing, extra)

        if not missing and not self.confirmed:
            self.confirmed  = True
            self.wait_count = 0
            print(f"  {'─' * 50}")
            print(f"  ALL LETTERS CONFIRMED — ready to place: {self.current_guess.upper()}")
            print(f"  {'─' * 50}")
            print(f"  Press Enter to submit G/B/I feedback.")
            print(f"  Type 'reset' to clear buffer, 'quit' to exit.\n")

    def _print_live_status(self, matched, missing, extra):
        """Prints the three section status."""
        needed_collected = [l for l in matched if l not in extra]
        print(f"  Waiting for  : {' '.join(missing) if missing else 'none'}")
        print(f"  Collected    : {' '.join(needed_collected) if needed_collected else 'none'}")
        print(f"  Extra letters: {' '.join(extra) if extra else 'none'}\n")

    def print_status(self):
        """Print full letter collection status."""
        collected     = list(self.letter_buffer.keys())
        required_copy = self.required_letters[:]

        matched   = []
        missing   = []
        remaining = list(collected)

        for letter in required_copy:
            if letter in remaining:
                matched.append(letter)
                remaining.remove(letter)
            else:
                missing.append(letter)

        extra = remaining

        print(f"\n  {'─' * 50}")
        print(f"  Letter Collection Status")
        print(f"  {'─' * 50}")
        print(f"  Required     : {' '.join(self.required_letters)}")
        print(f"  Waiting for  : {' '.join(missing) if missing else 'none'}")
        print(f"  Collected    : {' '.join(matched) if matched else 'none'}")
        print(f"  Extra letters: {' '.join(extra) if extra else 'none'}")
        print(f"\n  Buffer details:")
        for letter, data in self.letter_buffer.items():
            needed = letter in self.required_letters
            tag    = "[NEEDED]" if needed else "[EXTRA]"
            print(f"    {tag} {letter}"
                  f"  conf={data['conf']:.1f}%"
                  f"  pos=({data.get('x_m')}, {data.get('y_m')}, {data.get('z_m')})")
        print(f"  {'─' * 50}\n")

    def reset_buffer(self):
        """Clear buffer and restart collection for current guess."""
        self.letter_buffer = {}
        self.prev_seen     = set()
        self.confirmed     = False
        self.wait_count    = 0
        print(f"\n  Buffer cleared. Place letters again.")
        print(f"  Required: {' '.join(self.required_letters)}\n")

    def submit_feedback(self, feedback_str):
        """Process G/B/I feedback and move to next guess."""
        try:
            feedback = parse_feedback(feedback_str)
        except ValueError as e:
            print(f"  Invalid feedback: {e}\n")
            return False

        if all(f == GOOD for f in feedback):
            print(f"\n  SOLVED in {self.attempt} attempt(s)!"
                  f"  Word: {self.current_guess.upper()}\n")
            return True

        self.candidates = filter_candidates(
            self.candidates, self.current_guess, feedback)

        print(f"\n  {len(self.candidates)} candidates remaining.")

        if not self.candidates:
            print(f"  No candidates left.\n")
            return True

        self.attempt += 1
        self.next_guess()
        return False


# ─────────────────────────────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────────────────────────────

class PerceptionTestNode(Node):

    def __init__(self, game_state):
        super().__init__('gamification_perception_test')
        self.game_state = game_state

        self.create_subscription(
            String,
            '/perception/detections',
            self.detections_callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /perception/detections — waiting for letters...')

    def detections_callback(self, msg):
        """Receive detection frame from Luke's perception node."""
        try:
            data   = json.loads(msg.data)
            blocks = data.get('blocks', [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detections JSON: {e}')
            return

        if not blocks:
            return

        self.game_state.process_detections(blocks)


# ─────────────────────────────────────────────────────────────────
#  Waiting countdown — background thread
# ─────────────────────────────────────────────────────────────────

def waiting_countdown(state):
    """Prints waiting status every 3 seconds while letters are missing."""
    import time
    while state.running:
        time.sleep(3)
        if state.confirmed:
            continue

        collected     = list(state.letter_buffer.keys())
        required_copy = state.required_letters[:]

        matched   = []
        missing   = []
        remaining = list(collected)

        for letter in required_copy:
            if letter in remaining:
                matched.append(letter)
                remaining.remove(letter)
            else:
                missing.append(letter)

        extra = remaining

        if not missing:
            continue

        state.wait_count += 3
        needed_collected = [l for l in matched if l not in extra]
        print(f"  [{state.wait_count:>3}s] Waiting for  : {' '.join(missing) if missing else 'none'}")
        print(f"         Collected    : {' '.join(needed_collected) if needed_collected else 'none'}")
        print(f"         Extra letters: {' '.join(extra) if extra else 'none'}\n")


# ─────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────

def main():
    print(f"\n  {'=' * 50}")
    print(f"  Gamification <-> Perception Test (ROS2)")
    print(f"  {'=' * 50}")
    print(f"  Subscribing to /perception/detections")
    print(f"  Place letters under the camera one at a time.\n")
    print(f"  Commands:")
    print(f"    Enter     = submit G/B/I feedback after confirmed guess")
    print(f"    show      = show current status")
    print(f"    reset     = reset letter buffer")
    print(f"    quit      = quit")
    print(f"  {'=' * 50}\n")

    dict_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
    words     = load_dictionary(dict_path)

    state = GameState(words)

    rclpy.init()
    node = PerceptionTestNode(state)

    # Pick first guess
    state.next_guess()

    # Spin ROS2 in background thread so keyboard input works in main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Waiting countdown in background thread
    countdown_thread = threading.Thread(
        target=waiting_countdown, args=(state,), daemon=True)
    countdown_thread.start()

    try:
        while rclpy.ok():
            user_input = input().strip().lower()

            if user_input == 'quit':
                print(f"\n  Goodbye.\n")
                break

            elif user_input == 'reset':
                state.reset_buffer()

            elif user_input == 'show':
                state.print_status()

            elif user_input == '':
                if not state.confirmed:
                    missing = [l for l in state.required_letters
                               if l not in state.letter_buffer]
                    print(f"  Not all letters confirmed yet.")
                    print(f"  Still missing: {missing}\n")
                else:
                    print(f"  Enter feedback (e.g. G B I G G or GGGGG): ", end='', flush=True)
                    feedback_str = input().strip()
                    done = state.submit_feedback(feedback_str)
                    if done:
                        break

            else:
                print(f"  Commands: Enter=feedback  show=status  reset=clear  quit=exit\n")

    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        node.destroy_node()
        rclpy.shutdown()
        print(f"\n  Session ended.\n")


if __name__ == '__main__':
    main()