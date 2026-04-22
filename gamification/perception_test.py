#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  perception_test.py
#
#  Standalone test of gamification letter collection logic.
#  NO ROS2 required — runs directly in PowerShell/terminal.
#
#  Simulates Luke's perception node by letting you type letters
#  as if the camera detected them, one at a time.
#
#  HOW TO RUN:
#    python perception_test.py
#
#  CONTROLS:
#    [letter]  — simulate camera detecting that letter
#    Enter     — submit G/B/I feedback after confirmed guess
#    show      — show current status
#    reset     — reset letter buffer
#    quit      — quit
# ─────────────────────────────────────────────────────────────────

import os
import sys
import time
import threading

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
        self.letter_buffer    = {}
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
        self.confirmed        = False
        self.wait_count       = 0

        print(f"\n  {'=' * 50}")
        print(f"  Attempt {self.attempt}")
        print(f"  Target word : {self.current_guess.upper()}")
        print(f"  Place these letters under the camera:")
        print(f"    {' '.join(self.required_letters)}")
        print(f"  {'=' * 50}")
        print(f"  (type a letter to simulate camera detection)\n")

    def detect_letter(self, letter):
        """Simulate camera detecting a letter."""
        letter = letter.upper()

        if not letter.isalpha() or len(letter) != 1:
            print("  Invalid input — type a single letter.\n")
            return

        if letter in self.letter_buffer:
            print(f"  {letter} already in buffer — skipping.\n")
            return

        self.letter_buffer[letter] = {
            'letter'   : letter,
            'conf'     : 95.0,
            'x_m'      : round(len(self.letter_buffer) * 0.05, 3),
            'y_m'      : 0.0,
            'z_m'      : 0.25,
            'theta_deg': 0.0,
        }
        self.wait_count = 0

        print(f"  + Letter detected: {letter}"
              f"  (conf=95.0%"
              f"  pos=({self.letter_buffer[letter]['x_m']}, 0.0, 0.25))")

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
        """Prints the three section status — matched/missing/extra are separate."""
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
                  f"  pos=({data['x_m']}, {data['y_m']}, {data['z_m']})")
        print(f"  {'─' * 50}\n")

    def reset_buffer(self):
        """Clear buffer and restart collection."""
        self.letter_buffer = {}
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
#  Waiting countdown — runs in background thread
# ─────────────────────────────────────────────────────────────────

def waiting_countdown(state):
    """Prints waiting status every 3 seconds while letters are missing."""
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
    print(f"  Gamification <-> Perception Test")
    print(f"  {'=' * 50}")
    print(f"  Simulating perception — type letters as if")
    print(f"  the camera detected them one at a time.\n")
    print(f"  Commands:")
    print(f"    [letter]  = simulate camera detecting that letter")
    print(f"    Enter     = submit G/B/I feedback after confirmed guess")
    print(f"    show      = show current status")
    print(f"    reset     = reset letter buffer")
    print(f"    quit      = quit")
    print(f"  {'=' * 50}\n")

    dict_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary.txt')
    words     = load_dictionary(dict_path)

    state = GameState(words)
    state.next_guess()

    countdown_thread = threading.Thread(
        target=waiting_countdown, args=(state,), daemon=True)
    countdown_thread.start()

    try:
        while True:
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

            elif len(user_input) == 1 and user_input.isalpha():
                state.detect_letter(user_input)

            else:
                print(f"  Commands: [letter]=detect  Enter=feedback  show=status  reset=clear  quit=exit\n")

    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        print(f"\n  Session ended.\n")


if __name__ == '__main__':
    main()