# ─────────────────────────────────────────────
#  voice_control/main.py
#
#  Voice recognition controller for the Wordle
#  gamification node. Listens to mic input and
#  maps speech to game commands.
#
#  ROS2 note:
#    listen()            -> subscriber callback (audio transcript)
#    publish_command()   -> publisher (command events)
#    The main loop       -> rclpy.spin()
# ─────────────────────────────────────────────

import sys
import os
import tempfile
import speech_recognition as sr

# ── Add gamification folder to path so we can import it ──
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'gamification'))
from speaker_verification import run_speaker_verification_startup
from constants    import GOOD, BAD_POSITION, INCORRECT, EASTER_EGG_WORDS
from dictionary   import load_dictionary
from wordle_logic import (
    score_guess_against_target,
    filter_candidates,
    choose_opening_guess,
    choose_best_guess,
)
from display import (
    colour_feedback,
    print_remaining_info,
    print_colour_legend,
    print_title,
    trigger_easter_egg,
    print_bug_report,
    update_stats,
    print_stats,
)


# ─────────────────────────────────────────────
#  Voice Engine Setup
# ─────────────────────────────────────────────

recognizer = sr.Recognizer()
recognizer.energy_threshold         = 300
recognizer.pause_threshold          = 0.8
recognizer.dynamic_energy_threshold = True


def listen(prompt=None, timeout=8):
    """
    Captures mic input using sounddevice and returns recognised text.
    Returns None if nothing was heard or recognition failed.

    ROS2 note: replace with subscriber callback on audio transcript topic.
    """
    import sounddevice as sd
    import numpy as np
    import scipy.io.wavfile as wav

    if prompt:
        print(f"\n  {prompt}")

    print("  >> Listening...", end="", flush=True)

    sample_rate = 16000
    duration    = 5

    try:
        recording = sd.rec(
            int(duration * sample_rate),
            samplerate=sample_rate,
            channels=1,
            dtype='int16'
        )
        sd.wait()

        # Write to temp file — close handle first so Windows doesn't lock it
        tmp      = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        tmp_path = tmp.name
        tmp.close()
        wav.write(tmp_path, sample_rate, recording)

        with sr.AudioFile(tmp_path) as source:
            audio = recognizer.record(source)

        try:
            os.unlink(tmp_path)
        except Exception:
            pass

        text = recognizer.recognize_google(audio).lower()
        print(f" heard: '{text}'")
        return text

    except sr.UnknownValueError:
        print(" (could not understand — nothing recognisable heard)")
        return None
    except sr.RequestError as e:
        print(f" (API error: {e})")
        return None
    except Exception as e:
        print(f" (error: {e})")
        return None


def publish_command(command):
    """
    Publishes a recognised command.
    Currently just prints — replace with ROS2 publisher later.

    ROS2 note: self.command_publisher.publish(String(data=command))
    """
    print(f"  [CMD] {command}")


# ─────────────────────────────────────────────
#  Speech -> Command Mapping
# ─────────────────────────────────────────────

MODE_COMMANDS = {
    "manual"    : "1",
    "one"       : "1",
    "mode one"  : "1",
    "auto"      : "2",
    "two"       : "2",
    "test"      : "2",
    "mode two"  : "2",
    "stats"     : "3",
    "three"     : "3",
    "statistics": "3",
    "quit"      : "4",
    "exit"      : "4",
    "four"      : "4",
    "bye"       : "4",
    "bug"       : "5",
    "five"      : "5",
    "debug"     : "5",
    "six"       : "6",
    "toggle"    : "6",
    "voice"     : "6",
}

FEEDBACK_WORDS = {
    # Green / correct
    "good"          : GOOD,
    "correct"       : GOOD,
    "yes"           : GOOD,
    "right"         : GOOD,
    "green"         : GOOD,
    # Yellow / bad position
    "bad"           : BAD_POSITION,
    "wrong position": BAD_POSITION,
    "misplaced"     : BAD_POSITION,
    "yellow"        : BAD_POSITION,
    "close"         : BAD_POSITION,
    "move"          : BAD_POSITION,
    # Grey / incorrect
    "incorrect"     : INCORRECT,
    "no"            : INCORRECT,
    "wrong"         : INCORRECT,
    "grey"          : INCORRECT,
    "gray"          : INCORRECT,
    "nope"          : INCORRECT,
    "not in"        : INCORRECT,
}


def parse_mode_command(text):
    """Converts spoken text to a mode number string. Returns None if no match."""
    if text is None:
        return None
    text = text.lower().strip()
    for phrase, mode in MODE_COMMANDS.items():
        if phrase in text:
            return mode
    return None


def parse_feedback_command(text):
    """
    Converts spoken feedback into a list of 5 feedback tokens.
    Say: 'good bad incorrect good good'
    Returns: [GOOD, BAD_POSITION, INCORRECT, GOOD, GOOD]
    Returns None if can't parse 5 tokens.
    """
    if text is None:
        return None

    found = []
    words = text.lower().strip().split()
    i     = 0

    while i < len(words) and len(found) < 5:
        matched = False

        # Try two-word phrases first (e.g. "wrong position")
        if i + 1 < len(words):
            two_word = words[i] + " " + words[i + 1]
            if two_word in FEEDBACK_WORDS:
                found.append(FEEDBACK_WORDS[two_word])
                i += 2
                matched = True

        if not matched and words[i] in FEEDBACK_WORDS:
            found.append(FEEDBACK_WORDS[words[i]])
            i += 1
            matched = True

        if not matched:
            i += 1

    return found if len(found) == 5 else None


# ─────────────────────────────────────────────
#  Voice Input Wrappers
# ─────────────────────────────────────────────

def voice_mode_select(voice_enabled, retries=3):
    """
    Returns a mode string "1"-"6".
    If voice is off, falls straight to keyboard input.
    """
    if not voice_enabled:
        return input("  Choose mode: ").strip()

    print("\n  Say a mode: 'manual', 'auto', 'stats', 'quit', 'bug', or 'toggle'")

    for attempt in range(retries):
        text = listen()
        mode = parse_mode_command(text)

        if mode:
            publish_command(f"MODE_{mode}")
            return mode

        remaining = retries - attempt - 1
        if remaining > 0:
            if text:
                print(f"  Heard '{text}' but didn't match any command. {remaining} attempt(s) left.")
            else:
                print(f"  Nothing heard. {remaining} attempt(s) left.")

    print("  Voice failed. Type your choice: ", end="")
    return input().strip()


def voice_feedback(voice_enabled, retries=3):
    """
    Returns a list of 5 feedback tokens.
    If voice is off, falls straight to keyboard input.
    """
    if not voice_enabled:
        print("  Type feedback (e.g. G B I G G): ", end="")
        raw     = input().strip()
        mapping = {"G": GOOD, "B": BAD_POSITION, "I": INCORRECT}
        cleaned = raw.upper().replace(" ", "").replace(",", "")
        if len(cleaned) == 5 and all(c in mapping for c in cleaned):
            return [mapping[c] for c in cleaned]
        return None

    print("  Say feedback for each letter: e.g. 'good bad incorrect good good'")

    for attempt in range(retries):
        text     = listen()
        feedback = parse_feedback_command(text)

        if feedback:
            readable = " ".join(
                "G" if f == GOOD else "B" if f == BAD_POSITION else "I"
                for f in feedback
            )
            publish_command(f"FEEDBACK_{readable}")
            print(f"  Parsed as: {readable}")
            return feedback

        remaining = retries - attempt - 1
        if remaining > 0:
            if text:
                print(f"  Heard '{text}' but couldn't parse 5 results. {remaining} attempt(s) left.")
            else:
                print(f"  Nothing heard. {remaining} attempt(s) left.")
            print("  Try saying: 'good bad incorrect good good'")

    # Fallback to keyboard
    print("  Voice failed. Type feedback (e.g. G B I G G): ", end="")
    raw     = input().strip()
    mapping = {"G": GOOD, "B": BAD_POSITION, "I": INCORRECT}
    cleaned = raw.upper().replace(" ", "").replace(",", "")
    if len(cleaned) == 5 and all(c in mapping for c in cleaned):
        return [mapping[c] for c in cleaned]
    return None


# ─────────────────────────────────────────────
#  Voice-Controlled Game Modes
# ─────────────────────────────────────────────

def voice_manual_solver(words, voice_enabled):
    """Manual mode — script guesses, you provide feedback via voice or keyboard."""
    candidates = words[:]
    attempt    = 1

    print("\n  Think of a 5-letter word. DO NOT say it out loud.")
    print("  The script will guess. You provide the feedback.\n")
    print_colour_legend()
    if voice_enabled:
        print("  Say: 'good', 'bad', 'incorrect' for each letter.\n")
    else:
        print("  Type: G (good), B (bad position), I (incorrect)\n")

    while True:
        if not candidates:
            print("\n  No candidates left. Your feedback may have been inconsistent.")
            return

        guess = choose_opening_guess(words) if attempt == 1 else choose_best_guess(candidates)

        print(f"\n  ── Attempt {attempt} ──")
        print(f"  Guess: {guess.upper()}")

        feedback = voice_feedback(voice_enabled)

        if feedback is None:
            print("  Could not get valid feedback. Try again.")
            continue

        print(colour_feedback(guess, feedback))

        if all(f == GOOD for f in feedback):
            print(f"\n  Solved in {attempt} attempt(s)! The word was: {guess.upper()}")
            update_stats(attempt)
            return

        candidates = filter_candidates(candidates, guess, feedback)
        print_remaining_info(candidates)
        attempt += 1


def voice_auto_solver(words, voice_enabled):
    """Auto test mode — speak or type the target word, script solves it."""
    if voice_enabled:
        print("\n  Say the 5-letter target word for testing:")
        text = listen(timeout=6)
        if text is None or not text.replace(" ", "").isalpha():
            print("  Couldn't catch a valid word. Type it instead: ", end="")
            text = input().strip().lower()
    else:
        text = input("\n  Enter the hidden 5-letter word for testing: ").strip().lower()

    target = text.strip().lower()

    if not target.isalpha() or len(target) != 5:
        print("  Please enter exactly 5 letters.")
        return

    if target not in words:
        print(f"  '{target}' is not in dictionary.txt")
        return

    if target in EASTER_EGG_WORDS:
        trigger_easter_egg(target)

    candidates = words[:]
    attempt    = 1

    print(f"\n  Testing solver against: {target.upper()}\n")

    while True:
        if not candidates:
            print("  No candidates left -- something went wrong.")
            return

        guess    = choose_opening_guess(words) if attempt == 1 else choose_best_guess(candidates)
        feedback = score_guess_against_target(guess, target)

        print(f"  Attempt {attempt}: {guess.upper()}  ->  {colour_feedback(guess, feedback).strip()}")

        if guess == target:
            print(f"\n  Solved in {attempt} attempt(s)!")
            update_stats(attempt)
            return

        candidates = filter_candidates(candidates, guess, feedback)
        print_remaining_info(candidates)
        attempt += 1


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────

def main():
    dict_path = os.path.join(os.path.dirname(__file__), '..', 'gamification', 'dictionary.txt')
    words     = load_dictionary(dict_path)

    print("\n  Voice Control Node — Wordle Solver")

    # ── Speaker verification ──
    player_name, verified, voice_features = run_speaker_verification_startup()
    main.voice_features = voice_features

    if verified:
        print(f"  Session locked to: {player_name}")
        print("  Other voices will be rejected during gameplay.\n")
    elif player_name:
        print(f"  Playing as {player_name} (unverified — anyone can speak).\n")
    else:
        print("  No player registered — open session.\n")

    main.voice_enabled = False
    main.player_name   = player_name
    main.verified      = verified

    while True:
        voice_status = "ON" if main.voice_enabled else "OFF"

        print_title()
        print("  1 = Manual mode  (you think of a word, script guesses)")
        print("  2 = Auto test    (script plays against a known word)")
        print("  3 = Stats        (session performance)")
        print("  4 = Quit")
        print("  5 = Bug")
        print(f"  6 = Toggle voice [{voice_status}]\n")

        mode = voice_mode_select(main.voice_enabled)

        if mode == "1":
            voice_manual_solver(words, main.voice_enabled)
        elif mode == "2":
            voice_auto_solver(words, main.voice_enabled)
        elif mode == "3":
            print_stats()
        elif mode == "4":
            print_stats()
            print("\n  Goodbye.\n")
            break
        elif mode == "5":
            print_bug_report()
        elif mode == "6":
            main.voice_enabled = not main.voice_enabled
            status = "ON" if main.voice_enabled else "OFF"
            print(f"\n  Voice input turned {status}.\n")
        else:
            print("  Invalid choice. Pick 1 through 6.")


if __name__ == "__main__":
    main()
