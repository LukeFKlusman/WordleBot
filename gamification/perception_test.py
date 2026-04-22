#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  perception_test.py
#
#  Standalone test — subscribes to Luke's /perception/detections
#  and runs the gamification letter collection logic.
#
#  No GUI, no SS2 — just terminal output to verify letters are
#  being collected correctly before full integration.
#
#  HOW TO RUN:
#    Terminal 1: Luke runs his perception node
#    Terminal 2: python3 gamification/perception_test.py
#
#  CONTROLS (type in terminal):
#    Enter  — submit G/B/I feedback for confirmed guess
#    s      — show current letter collection status
#    r      — reset letter buffer
#    q      — quit
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
#  Terminal colours
# ─────────────────────────────────────────────────────────────────

class C:
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    CYAN   = "\033[96m"
    BOLD   = "\033[1m"
    DIM    = "\033[2m"
    RESET  = "\033[0m"


def print_status(required, collected, missing, extra, buffer):
    """Prints current letter collection status to terminal."""
    print(f"\n  {'─' * 50}")
    print(f"  {C.BOLD}Letter Collection Status{C.RESET}")
    print(f"  {'─' * 50}")
    print(f"  Required  : {C.CYAN}{' '.join(required)}{C.RESET}")
    print(f"  Collected : {C.GREEN}{' '.join(collected) if collected else '(none yet)'}{C.RESET}")
    print(f"  Missing   : {C.YELLOW}{' '.join(missing) if missing else '(none)'}{C.RESET}")

    if extra:
        print(f"  {C.RED}Extra (not needed): {' '.join(extra)}{C.RESET}")

    print(f"\n  Buffer contents:")
    for letter, data in buffer.items():
        needed = letter in required
        tag    = f"{C.GREEN}[NEEDED]{C.RESET}" if needed else f"{C.RED}[EXTRA]{C.RESET}"
        x      = data.get('x_m', 'N/A')
        y      = data.get('y_m', 'N/A')
        z      = data.get('z_m', 'N/A')
        conf   = data.get('conf', 0)
        print(f"    {tag} {C.BOLD}{letter}{C.RESET}  conf={conf:.1f}%  pos=({x}, {y}, {z})")

    if not missing:
        print(f"\n  {C.GREEN}{C.BOLD}✓ All required letters collected!{C.RESET}")
    else:
        print(f"\n  Still waiting for: {C.YELLOW}{' '.join(missing)}{C.RESET}")

    print(f"  {'─' * 50}\n")


# ─────────────────────────────────────────────────────────────────
#  ROS2 Subscriber Node
# ─────────────────────────────────────────────────────────────────

class PerceptionTestNode(Node):

    def __init__(self, words):
        super().__init__('gamification_perception_test')

        self.words            = words
        self.candidates       = words[:]
        self.attempt          = 1
        self.current_guess    = None
        self.required_letters = []
        self.letter_buffer    = {}
        self.prev_seen        = set()
        self.confirmed        = False
        self.wait_count       = 0  # seconds elapsed waiting for letters

        # Subscribe to Luke's perception topic
        self.create_subscription(
            String,
            '/perception/detections',
            self.detections_callback,
            10
        )

        # Waiting countdown — fires every second
        self.create_timer(1.0, self.waiting_timer_callback)

        self.get_logger().info(
            'Perception test node ready.\n'
            'Subscribed to /perception/detections\n'
            'Waiting for detections from Luke\'s node...'
        )

        # Pick first guess immediately
        self._next_guess()


    # ─────────────────────────────────────────────────────────────
    #  Waiting countdown timer
    # ─────────────────────────────────────────────────────────────

    def waiting_timer_callback(self):
        """
        Fires every second. Prints a waiting message if letters
        are still missing and nothing new has been detected recently.
        """
        if self.confirmed:
            return

        collected = list(self.letter_buffer.keys())
        missing   = [l for l in self.required_letters if l not in collected]

        if not missing:
            return

        self.wait_count += 1

        print(f"  {C.DIM}[{self.wait_count:>3}s] Waiting for: "
              f"{C.YELLOW}{' '.join(missing)}{C.RESET}"
              f"{C.DIM}  |  collected: "
              f"{C.GREEN}{' '.join(collected) if collected else 'none'}{C.RESET}")


    # ─────────────────────────────────────────────────────────────
    #  Detections callback
    # ─────────────────────────────────────────────────────────────

    def detections_callback(self, msg):
        """
        Receives detection frames from Luke's perception node.
        Stores new letters as they appear, flags wrong/extra ones.
        Letters that disappear stay stored in the buffer.
        """
        try:
            data   = json.loads(msg.data)
            blocks = data.get('blocks', [])
        except json.JSONDecodeError:
            return

        if not blocks:
            return

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

        # Store any new letters that appear this frame
        for letter in new_letters:
            if letter not in self.letter_buffer:
                self.letter_buffer[letter] = current_seen[letter]
                self.wait_count = 0  # reset waiting counter on new detection
                print(f"\n  {C.GREEN}+ New letter detected: "
                      f"{C.BOLD}{letter}{C.RESET}"
                      f"  (conf={current_seen[letter]['conf']:.1f}%)")
            else:
                # Already stored — update position with latest reading
                self.letter_buffer[letter] = current_seen[letter]

        self.prev_seen = current_set

        # Run match check
        if self.required_letters:
            self._check_match()


    # ─────────────────────────────────────────────────────────────
    #  Letter match check
    # ─────────────────────────────────────────────────────────────

    def _check_match(self):
        """Check collected letters against required. Flag issues."""
        collected     = list(self.letter_buffer.keys())
        required_copy = self.required_letters[:]

        # Check each required letter (handles duplicates)
        matched   = []
        missing   = []
        remaining = list(collected)

        for letter in required_copy:
            if letter in remaining:
                matched.append(letter)
                remaining.remove(letter)
            else:
                missing.append(letter)

        extra = remaining  # leftover letters not needed

        if extra:
            print(f"  {C.RED}⚠ Extra letters detected "
                  f"(not needed): {' '.join(extra)}{C.RESET}")

        # All collected — confirm
        if not missing and not self.confirmed:
            self.confirmed  = True
            self.wait_count = 0
            print_status(
                self.required_letters, collected, missing, extra, self.letter_buffer)
            print(f"  {C.GREEN}{C.BOLD}✓ CONFIRMED — ready to place: "
                  f"{self.current_guess.upper()}{C.RESET}\n")
            print(f"  Press Enter to submit G/B/I feedback,")
            print(f"  or type 'r' to reset buffer, 'q' to quit.\n")


    # ─────────────────────────────────────────────────────────────
    #  Next guess
    # ─────────────────────────────────────────────────────────────

    def _next_guess(self):
        """Pick next best guess and set required letters."""
        if self.attempt == 1:
            self.current_guess = choose_opening_guess(self.words)
        else:
            self.current_guess = choose_best_guess(self.candidates)

        if not self.current_guess:
            print(f"\n  {C.RED}No valid guess available.{C.RESET}\n")
            return

        self.required_letters = list(self.current_guess.upper())
        self.letter_buffer    = {}
        self.prev_seen        = set()
        self.confirmed        = False
        self.wait_count       = 0

        print(f"\n  {'═' * 50}")
        print(f"  {C.BOLD}Attempt {self.attempt}{C.RESET}")
        print(f"  Target word : {C.CYAN}{C.BOLD}{self.current_guess.upper()}{C.RESET}")
        print(f"  Place these letters under the camera:")
        print(f"  {C.YELLOW}{C.BOLD}  {' '.join(self.required_letters)}{C.RESET}")
        print(f"  {'═' * 50}\n")


    # ─────────────────────────────────────────────────────────────
    #  Buffer reset
    # ─────────────────────────────────────────────────────────────

    def reset_buffer(self):
        """Clears letter buffer and restarts collection for current guess."""
        self.letter_buffer = {}
        self.prev_seen     = set()
        self.confirmed     = False
        self.wait_count    = 0
        print(f"\n  {C.YELLOW}Buffer cleared. Place letters again.{C.RESET}")
        print(f"  Required: {C.CYAN}{' '.join(self.required_letters)}{C.RESET}\n")


    # ─────────────────────────────────────────────────────────────
    #  Feedback submission
    # ─────────────────────────────────────────────────────────────

    def submit_feedback(self, feedback_str):
        """Process G/B/I feedback and move to next guess."""
        try:
            feedback = parse_feedback(feedback_str)
        except ValueError as e:
            print(f"  {C.RED}Invalid feedback: {e}{C.RESET}\n")
            return False

        if all(f == GOOD for f in feedback):
            print(f"\n  {C.GREEN}{C.BOLD}✓ SOLVED in {self.attempt} attempt(s)! "
                  f"Word: {self.current_guess.upper()}{C.RESET}\n")
            return True  # signal quit

        self.candidates = filter_candidates(
            self.candidates, self.current_guess, feedback)

        print(f"\n  {len(self.candidates)} candidates remaining.")

        if not self.candidates:
            print(f"  {C.RED}No candidates left — something went wrong.{C.RESET}\n")
            return True

        self.attempt += 1
        self._next_guess()
        return False


# ─────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────

def main():
    print(f"\n  {C.BOLD}┌───────────────────────────────────┐{C.RESET}")
    print(f"  {C.BOLD}│  Gamification ↔ Perception Test   │{C.RESET}")
    print(f"  {C.BOLD}└───────────────────────────────────┘{C.RESET}\n")
    print(f"  Subscribing to /perception/detections")
    print(f"  Place letters under the camera one at a time.\n")
    print(f"  Commands:")
    print(f"    Enter = submit feedback after confirmed guess")
    print(f"    s     = show current status")
    print(f"    r     = reset letter buffer")
    print(f"    q     = quit\n")

    dict_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
    words     = load_dictionary(dict_path)

    rclpy.init()
    node = PerceptionTestNode(words)

    # Spin ROS2 in background thread so keyboard input works in main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            user_input = input().strip().lower()

            if user_input == 'q':
                print(f"\n  {C.DIM}Goodbye.{C.RESET}\n")
                break

            elif user_input == 'r':
                node.reset_buffer()

            elif user_input == 's':
                collected = list(node.letter_buffer.keys())
                required  = node.required_letters
                missing   = [l for l in required if l not in collected]
                extra     = [l for l in collected if l not in required]
                print_status(required, collected, missing, extra, node.letter_buffer)

            elif user_input == '':
                if not node.confirmed:
                    missing = [l for l in node.required_letters
                               if l not in node.letter_buffer]
                    print(f"  {C.YELLOW}Not all letters confirmed yet.{C.RESET}")
                    print(f"  Still missing: {missing}\n")
                else:
                    print(f"  Enter feedback (e.g. G B I G G or GGGGG): ", end='', flush=True)
                    feedback_str = input().strip()
                    done = node.submit_feedback(feedback_str)
                    if done:
                        break

            else:
                print(f"  Commands: Enter=feedback  s=status  r=reset  q=quit\n")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()