#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  voice_node.py
#
#  ROS2 voice control node — translates speech (or keyboard) into
#  feedback for the gamification node.
#
#  TOPICS SUBSCRIBED:
#    /gamification/guess   std_msgs/String  — current guess word
#    /mission/state        std_msgs/String  — START/STOP/RESET
#
#  TOPICS PUBLISHED:
#    /gamification/feedback  std_msgs/String  — G/B/I string (e.g. "GBIGB")
#    /mission/state          std_msgs/String  — voice-triggered commands
#
#  HOW TO RUN:
#    python3 voice_control/voice_node.py
# ─────────────────────────────────────────────────────────────────

import os
import sys
import time
import tempfile
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'gamification'))

from speaker_verification import run_speaker_verification_startup
from constants import GOOD, BAD_POSITION, INCORRECT


# ── Voice Engine ──────────────────────────────────────────────────

_recognizer                         = sr.Recognizer()
_recognizer.energy_threshold        = 300
_recognizer.pause_threshold         = 0.8
_recognizer.dynamic_energy_threshold = True

FEEDBACK_WORDS = {
    "good"          : GOOD,
    "correct"       : GOOD,
    "yes"           : GOOD,
    "right"         : GOOD,
    "green"         : GOOD,
    "bad"           : BAD_POSITION,
    "wrong position": BAD_POSITION,
    "misplaced"     : BAD_POSITION,
    "yellow"        : BAD_POSITION,
    "close"         : BAD_POSITION,
    "move"          : BAD_POSITION,
    "incorrect"     : INCORRECT,
    "no"            : INCORRECT,
    "wrong"         : INCORRECT,
    "grey"          : INCORRECT,
    "gray"          : INCORRECT,
    "nope"          : INCORRECT,
    "not in"        : INCORRECT,
}

MISSION_COMMANDS = {
    "start"  : "START",
    "begin"  : "START",
    "go"     : "START",
    "stop"   : "STOP",
    "pause"  : "STOP",
    "reset"  : "RESET",
    "restart": "RESET",
}


def listen():
    """Captures mic input and returns recognised text, or None on failure."""
    import sounddevice as sd
    import numpy as np
    import scipy.io.wavfile as wav

    sample_rate = 16000
    duration    = 5

    print("  >> Listening...", end="", flush=True)

    try:
        recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
        sd.wait()

        tmp      = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        tmp_path = tmp.name
        tmp.close()
        wav.write(tmp_path, sample_rate, recording)

        with sr.AudioFile(tmp_path) as source:
            audio = _recognizer.record(source)

        try:
            os.unlink(tmp_path)
        except Exception:
            pass

        text = _recognizer.recognize_google(audio).lower()
        print(f" heard: '{text}'")
        return text

    except sr.UnknownValueError:
        print(" (nothing recognisable)")
        return None
    except sr.RequestError as e:
        print(f" (Google API error: {e})")
        return None
    except Exception as e:
        print(f" (error: {e})")
        return None


def parse_feedback_voice(text):
    """
    Converts spoken text to a list of 5 feedback tokens.
    Say: 'good bad incorrect good good'  ->  [GOOD, BAD_POSITION, INCORRECT, GOOD, GOOD]
    Returns None if 5 tokens cannot be parsed.
    """
    if text is None:
        return None

    found = []
    words = text.lower().strip().split()
    i     = 0

    while i < len(words) and len(found) < 5:
        matched = False
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


def parse_mission_command(text):
    """Converts spoken text to a mission command string. Returns None if no match."""
    if text is None:
        return None
    for phrase, cmd in MISSION_COMMANDS.items():
        if phrase in text.lower():
            return cmd
    return None


def keyboard_feedback():
    """Reads G/B/I feedback from keyboard. Returns list of 5 tokens or None."""
    mapping = {"G": GOOD, "B": BAD_POSITION, "I": INCORRECT}
    raw     = input("  Type feedback (e.g. GBIGB or G B I G B): ").strip()
    cleaned = raw.upper().replace(" ", "").replace(",", "")
    if len(cleaned) == 5 and all(c in mapping for c in cleaned):
        return [mapping[c] for c in cleaned]
    print("  Invalid format — use 5 characters: G, B, or I.")
    return None


# ── ROS2 Node ─────────────────────────────────────────────────────

class VoiceControlNode(Node):

    def __init__(self, voice_enabled, player_name, saved_features):
        super().__init__('voice_control_node')

        self.voice_enabled   = voice_enabled
        self.player_name     = player_name
        self.saved_features  = saved_features

        self.current_guess     = None
        self.game_active       = False
        self.awaiting_feedback = False

        self.create_subscription(String, '/gamification/guess', self.guess_callback,   10)
        self.create_subscription(String, '/mission/state',      self.mission_callback, 10)

        self.pub_feedback = self.create_publisher(String, '/gamification/feedback', 10)
        self.pub_mission  = self.create_publisher(String, '/mission/state',         10)

        # Feedback collection runs in background so rclpy.spin() is never blocked
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info(
            '\nVoice control node ready.'
            f'\n  Voice input : {"ON" if voice_enabled else "OFF (keyboard fallback)"}'
            f'\n  Player      : {player_name or "open session"}'
            '\n  Subscribing : /gamification/guess  /mission/state'
            '\n  Publishing  : /gamification/feedback  /mission/state'
        )


    # ── Subscriber Callbacks ──────────────────────────────────────

    def guess_callback(self, msg):
        """New guess published — prompt the human for feedback."""
        self.current_guess     = msg.data.upper()
        self.awaiting_feedback = True
        print(f"\n  ── Guess: {self.current_guess} ──")
        if self.voice_enabled:
            print("  Say feedback for each letter: e.g. 'good bad incorrect good good'")
        else:
            print("  Type feedback (e.g. GBIGB or G B I G B):")


    def mission_callback(self, msg):
        """Track game state so we ignore feedback when no game is running."""
        state = msg.data.upper().strip()
        if state == 'START':
            self.game_active = True
            self.get_logger().info('[Mission] Game started.')
        elif state in ('STOP', 'RESET'):
            self.game_active       = False
            self.awaiting_feedback = False
            self.current_guess     = None
            self.get_logger().info(f'[Mission] {state}.')


    # ── Input Loop ────────────────────────────────────────────────

    def _input_loop(self):
        """
        Runs in a background thread.
        Waits for awaiting_feedback, collects voice or keyboard input,
        then publishes the result to /gamification/feedback.
        """
        while rclpy.ok():
            if not self.awaiting_feedback:
                time.sleep(0.1)
                continue

            feedback = self._collect_feedback(retries=3)
            if feedback is None:
                print("  Could not get valid feedback — try again.")
                continue

            readable = ''.join(
                'G' if f == GOOD else 'B' if f == BAD_POSITION else 'I'
                for f in feedback
            )
            print(f"  Feedback sent: {readable}")

            msg      = String()
            msg.data = readable
            self.pub_feedback.publish(msg)
            self.awaiting_feedback = False


    def _collect_feedback(self, retries=3):
        """
        Tries to collect 5-token feedback via voice or keyboard.
        Also handles mid-input mission commands (start/stop/reset).
        Returns list of 5 tokens or None.
        """
        if not self.voice_enabled:
            return keyboard_feedback()

        for attempt in range(retries):
            text = listen()

            # Let mission commands be spoken mid-game
            cmd = parse_mission_command(text)
            if cmd:
                msg      = String()
                msg.data = cmd
                self.pub_mission.publish(msg)
                print(f"  Mission command spoken: {cmd}")
                return None

            feedback = parse_feedback_voice(text)
            if feedback:
                return feedback

            remaining = retries - attempt - 1
            if remaining > 0:
                if text:
                    print(f"  Heard '{text}' but couldn't parse 5 results. {remaining} attempt(s) left.")
                else:
                    print(f"  Nothing heard. {remaining} attempt(s) left.")
                print("  Try: 'good bad incorrect good good'")

        print("  Voice failed — falling back to keyboard.")
        return keyboard_feedback()


# ── Entry Point ───────────────────────────────────────────────────

def main(args=None):
    print("\n  Voice Control Node — Wordle Robot")

    player_name, verified, saved_features = run_speaker_verification_startup()

    if verified:
        print(f"  Session locked to: {player_name}")
    elif player_name:
        print(f"  Playing as {player_name} (open session).")
    else:
        print("  No player registered — open session.")

    rclpy.init(args=args)
    node = VoiceControlNode(
        voice_enabled  = True,
        player_name    = player_name,
        saved_features = saved_features,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
