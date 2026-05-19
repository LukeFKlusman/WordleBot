#!/usr/bin/env python3

import argparse
import difflib
import json
import queue
import re
import sys
import tempfile
import wave
from pathlib import Path

import numpy as np
import sounddevice as sd
import speech_recognition as sr


SAMPLE_RATE = 16000


def emit(event, **payload):
    message = {"event": event, **payload}
    sys.stdout.write(json.dumps(message) + "\n")
    sys.stdout.flush()


class VoiceBridge:
    def __init__(self, dictionary_path):
        self.dictionary = self._load_dictionary(dictionary_path)
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300
        self.recognizer.pause_threshold = 0.8
        self.recognizer.dynamic_energy_threshold = True
        self.stream = None
        self.audio_chunks = queue.Queue()

    def _load_dictionary(self, dictionary_path):
        path = Path(dictionary_path) if dictionary_path else None
        if path is None or not path.exists():
            return set()

        words = set()
        for line in path.read_text(encoding="utf-8").splitlines():
            word = line.strip().lower()
            if len(word) == 5 and word.isalpha():
                words.add(word)
        return words

    def start_recording(self):
        if self.stream is not None:
            raise RuntimeError("Recording is already in progress")

        while not self.audio_chunks.empty():
            self.audio_chunks.get_nowait()

        def callback(indata, frames, time_info, status):
            del frames, time_info
            if status:
                print(f"sounddevice status: {status}", file=sys.stderr, flush=True)
            self.audio_chunks.put(indata.copy())

        self.stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="int16",
            callback=callback,
        )
        self.stream.start()
        emit("recording_started")

    def stop_recording(self):
        if self.stream is None:
            raise RuntimeError("No recording is currently running")

        self.stream.stop()
        self.stream.close()
        self.stream = None

        chunks = []
        while not self.audio_chunks.empty():
            chunks.append(self.audio_chunks.get_nowait())

        if not chunks:
            return {"transcript": "", "guess": ""}

        audio = np.concatenate(chunks, axis=0).flatten()
        transcript = self._transcribe(audio)
        guess = self._best_guess(transcript)
        return {"transcript": transcript, "guess": guess}

    def cancel_recording(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None

        while not self.audio_chunks.empty():
            self.audio_chunks.get_nowait()

        emit("recording_cancelled")

    def _transcribe(self, audio):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
            temp_path = Path(handle.name)

        try:
            with wave.open(str(temp_path), "wb") as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(SAMPLE_RATE)
                wav_file.writeframes(audio.tobytes())

            with sr.AudioFile(str(temp_path)) as source:
                recorded_audio = self.recognizer.record(source)

            return self.recognizer.recognize_google(recorded_audio).lower().strip()
        finally:
            temp_path.unlink(missing_ok=True)

    def _best_guess(self, transcript):
        if not transcript:
            return ""

        tokens = re.findall(r"[a-z]+", transcript.lower())
        candidates = []

        for token in tokens:
            if len(token) == 5:
                candidates.append(token)

        joined = "".join(tokens)
        if len(joined) == 5:
            candidates.append(joined)

        initial_letters = "".join(token[0] for token in tokens if len(token) == 1)
        if len(initial_letters) == 5:
            candidates.append(initial_letters)

        compact = re.sub(r"[^a-z]", "", transcript.lower())
        if len(compact) == 5:
            candidates.append(compact)

        seen = set()
        ordered_candidates = []
        for candidate in candidates:
            if candidate and candidate not in seen:
                ordered_candidates.append(candidate)
                seen.add(candidate)

        for candidate in ordered_candidates:
            if candidate in self.dictionary:
                return candidate

        for candidate in ordered_candidates:
            if len(candidate) == 5:
                matches = difflib.get_close_matches(candidate, self.dictionary, n=1, cutoff=0.72)
                if matches:
                    return matches[0]

        for candidate in ordered_candidates:
            if len(candidate) == 5:
                return candidate

        return ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dictionary", default="")
    args = parser.parse_args()

    bridge = VoiceBridge(args.dictionary)
    emit("ready")

    for raw_line in sys.stdin:
        line = raw_line.strip()
        if not line:
            continue

        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            emit("error", message="Invalid helper command")
            continue

        command = payload.get("command", "")

        try:
            if command == "start_recording":
                bridge.start_recording()
            elif command == "stop_recording":
                result = bridge.stop_recording()
                emit("recording_result", **result)
            elif command == "cancel_recording":
                bridge.cancel_recording()
            elif command == "shutdown":
                break
            else:
                emit("error", message=f"Unknown helper command: {command}")
        except sr.UnknownValueError:
            emit("recording_result", transcript="", guess="")
        except sr.RequestError as exc:
            emit("error", message=f"Speech API error: {exc}")
        except Exception as exc:
            emit("error", message=str(exc))


if __name__ == "__main__":
    main()
