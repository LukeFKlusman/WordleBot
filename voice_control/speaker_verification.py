# ─────────────────────────────────────────────
#  speaker_verification.py
#
#  Registers a player's voiceprint and verifies
#  that future voice input matches the registered
#  player. Uses librosa for feature extraction
#  (no webrtcvad or pyaudio required).
#
#  ROS2 note:
#    register_player()  -> Service: Register Player
#    verify_speaker()   -> Service: Lock session to player
#    player_name        -> Publisher: Speaker ID
# ─────────────────────────────────────────────

import os
import json
import time
import numpy as np
import sounddevice as sd
import librosa

# ─────────────────────────────────────────────
#  Config
# ─────────────────────────────────────────────

SAMPLE_RATE      = 16000
RECORD_SECONDS   = 5        # length of each calibration sample
NUM_SAMPLES      = 3        # samples averaged to build voiceprint
VERIFY_SECONDS   = 4        # length of verification recording
VERIFY_THRESHOLD = 0.82     # similarity score 0-1, above = accepted
MAX_VERIFY_TRIES = 3        # retries allowed on verification failure

VOICEPRINT_FILE  = os.path.join(os.path.dirname(__file__), "voiceprint.json")


# ─────────────────────────────────────────────
#  Recording
# ─────────────────────────────────────────────

def record_sample(duration=RECORD_SECONDS):
    """
    Records audio from mic for a fixed duration.
    Returns float32 numpy array normalised to [-1, 1].
    """
    print(f"  Recording for {duration}s...", end="", flush=True)
    recording = sd.rec(
        int(duration * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype='int16'
    )
    sd.wait()
    print(" done.")
    return recording.flatten().astype(np.float32) / 32768.0


# ─────────────────────────────────────────────
#  Feature Extraction
#  We use MFCCs (Mel Frequency Cepstral Coefficients)
#  — the standard way to fingerprint a voice.
#  Each person's vocal tract shape produces a
#  unique MFCC pattern regardless of what they say.
# ─────────────────────────────────────────────

def extract_features(audio):
    """
    Extracts MFCC features from raw audio.
    Returns a 1D feature vector representing the voiceprint.
    """
    # 40 MFCC coefficients — good balance of detail vs noise
    mfccs = librosa.feature.mfcc(
        y=audio,
        sr=SAMPLE_RATE,
        n_mfcc=40
    )

    # Also grab delta (rate of change) for better accuracy
    delta  = librosa.feature.delta(mfccs)
    delta2 = librosa.feature.delta(mfccs, order=2)

    # Stack and average across time to get a fixed-size vector
    combined = np.vstack([mfccs, delta, delta2])
    return np.mean(combined, axis=1)


def cosine_similarity(a, b):
    """Returns similarity score between two feature vectors (0 to 1)."""
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    if norm_a == 0 or norm_b == 0:
        return 0.0
    return float(np.dot(a, b) / (norm_a * norm_b))


# ─────────────────────────────────────────────
#  Save / Load Voiceprint
# ─────────────────────────────────────────────

def save_voiceprint(player_name, features):
    """Saves player name and voiceprint feature vector to disk."""
    data = {
        "player_name": player_name,
        "features"   : features.tolist(),
    }
    with open(VOICEPRINT_FILE, "w") as f:
        json.dump(data, f)
    print(f"  Voiceprint saved for: {player_name}")


def load_voiceprint():
    """
    Loads saved voiceprint from disk.
    Returns (player_name, features) or (None, None) if not found.
    """
    if not os.path.exists(VOICEPRINT_FILE):
        return None, None
    try:
        with open(VOICEPRINT_FILE, "r") as f:
            data = json.load(f)
        return data["player_name"], np.array(data["features"], dtype=np.float32)
    except Exception as e:
        print(f"  Could not load voiceprint: {e}")
        return None, None


def delete_voiceprint():
    """Deletes the saved voiceprint file."""
    if os.path.exists(VOICEPRINT_FILE):
        os.remove(VOICEPRINT_FILE)
        print("  Voiceprint deleted.")
    else:
        print("  No voiceprint on file.")


# ─────────────────────────────────────────────
#  Registration
# ─────────────────────────────────────────────

def register_player():
    """
    Records NUM_SAMPLES voice samples, extracts features from each,
    averages them into a single voiceprint, and saves to disk.
    Returns (player_name, features) on success, (None, None) on failure.

    ROS2 note: expose as Service: Register Player
    """
    print("\n  ── Player Registration ─────────────────")
    player_name = input("  Enter your name: ").strip()

    if not player_name:
        print("  No name entered. Registration cancelled.")
        return None, None

    print(f"\n  Hi {player_name}! We will record {NUM_SAMPLES} voice samples.")
    print("  Speak naturally — say anything you like for each sample.")
    print("  e.g. 'Hello my name is Luke and I am registering my voice'\n")

    all_features = []

    for i in range(NUM_SAMPLES):
        print(f"  Sample {i + 1} of {NUM_SAMPLES}:")
        input("  Press Enter when ready, then speak...")

        try:
            audio    = record_sample(RECORD_SECONDS)
            features = extract_features(audio)
            all_features.append(features)
            print(f"  Sample {i + 1} captured successfully.\n")
        except Exception as e:
            print(f"  Error recording sample: {e}")
            return None, None

        if i < NUM_SAMPLES - 1:
            time.sleep(0.3)

    # Average all samples into one voiceprint
    voiceprint = np.mean(all_features, axis=0)
    save_voiceprint(player_name, voiceprint)

    print(f"\n  Registration complete! Welcome, {player_name}.")
    print("  ────────────────────────────────────────\n")
    return player_name, voiceprint


# ─────────────────────────────────────────────
#  Verification
# ─────────────────────────────────────────────

def verify_speaker(saved_features, player_name, tries=MAX_VERIFY_TRIES):
    """
    Records a short sample and compares to saved voiceprint.
    Returns True if verified, False if rejected.

    ROS2 note: expose as Service: Lock session to player
               publish result on Speaker ID topic
    """
    print(f"\n  ── Speaker Verification ────────────────")
    print(f"  Verifying you are {player_name}...")
    print("  Speak for a few seconds after the prompt.\n")

    for attempt in range(tries):
        print(f"  Attempt {attempt + 1} of {tries}:")
        input("  Press Enter when ready, then speak...")

        try:
            audio    = record_sample(VERIFY_SECONDS)
            features = extract_features(audio)
            score    = cosine_similarity(features, saved_features)

            print(f"  Similarity score: {score:.2f}  (threshold: {VERIFY_THRESHOLD:.2f})")

            if score >= VERIFY_THRESHOLD:
                print(f"\n  Verified! Welcome back, {player_name}.")
                print("  ────────────────────────────────────────\n")
                return True
            else:
                remaining = tries - attempt - 1
                if remaining > 0:
                    print(f"  Voice did not match. {remaining} attempt(s) remaining.\n")
                else:
                    print(f"\n  Verification failed. Access denied.")
                    print("  ────────────────────────────────────────\n")

        except Exception as e:
            print(f"  Error during verification: {e}")

    return False


def verify_listen(saved_features, player_name):
    """
    Quick mid-game verification — called inside listen() when session
    is locked. Records and checks without full retry loop.
    Returns True if voice matches, False if it doesn't.

    ROS2 note: hook into audio subscriber callback
    """
    try:
        audio    = record_sample(VERIFY_SECONDS)
        features = extract_features(audio)
        score    = cosine_similarity(features, saved_features)
        return score >= VERIFY_THRESHOLD, audio
    except Exception:
        return False, None


# ─────────────────────────────────────────────
#  Startup Flow
# ─────────────────────────────────────────────

def run_speaker_verification_startup():
    """
    Called at startup in main.py.
    Handles registration/verification flow.

    Returns:
        player_name : str or None
        verified    : True = session locked to this player
                      False = unverified / open session
        features    : numpy array or None (needed for mid-game checks)
    """
    print("\n  ── Speaker Verification System ─────────")

    saved_name, saved_features = load_voiceprint()

    # ── Returning player ──
    if saved_name is not None:
        print(f"  Registered player found: {saved_name}")
        print("  1 = Verify and lock session to this player")
        print("  2 = Skip verification (anyone can play)")
        print("  3 = Delete registration and start fresh\n")

        choice = input("  Choose: ").strip()

        if choice == "1":
            verified = verify_speaker(saved_features, saved_name)
            if verified:
                return saved_name, True, saved_features
            else:
                print("  Falling back to unverified mode.")
                return saved_name, False, None

        elif choice == "3":
            delete_voiceprint()
            return run_speaker_verification_startup()

        else:
            print("  Skipping verification — anyone can play.\n")
            return saved_name, False, None

    # ── New player ──
    else:
        print("  No registered player found.")
        print("  1 = Register your voice now")
        print("  2 = Skip (anyone can play)\n")

        choice = input("  Choose: ").strip()

        if choice == "1":
            name, features = register_player()
            if name:
                return name, True, features
            else:
                return None, False, None
        else:
            print("  Skipping registration — anyone can play.\n")
            return None, False, None
