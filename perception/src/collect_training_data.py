"""
collect_training_data.py
════════════════════════════════════════════════════════════
Collect labelled training images from your RealSense camera or webcam.

HOW TO USE:
  python3 collect_training_data.py            ← prompts you to choose camera
  python3 collect_training_data.py realsense  ← use RealSense directly
  python3 collect_training_data.py webcam     ← use webcam directly

TIPS:
  - Move/tilt the card between EVERY keypress for variation
  - Watch the Crop Preview window — that's exactly what the CNN sees
  - Aim for 60-80 keypresses per letter with genuine variation

OUTPUT:
  ../data/raw/
    A/  → 001.png, 002.png ...
    B/  → 001.png, 002.png ...
    ...
"""

import cv2
import numpy as np
import os
import sys
import time

_HERE      = os.path.dirname(os.path.abspath(__file__))
SAVE_ROOT  = os.path.join(_HERE, "../data/raw")
BURST_SIZE = 4       # One image per keypress — YOU control variation by moving card
BURST_DELAY = 0.05
IMG_SIZE   = 64

LABEL_MAP = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")

# ── Camera selection ───────────────────────────────────────
def select_camera():
    # Check command line argument first
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg == "realsense":
            return "realsense"
        elif arg == "webcam":
            return "webcam"

    # Otherwise prompt
    print("\n══════════════════════════════════════════════")
    print("  Select camera:")
    print("  1 = RealSense")
    print("  2 = Webcam (laptop camera)")
    print("══════════════════════════════════════════════")
    choice = input("  Enter 1 or 2: ").strip()
    return "realsense" if choice == "1" else "webcam"

def preprocess(frame):
    """
    Find the white card in the frame, crop the inner letter area (cuts white border),
    convert to greyscale, resize to 64x64.
    This matches exactly what realsense_camera_cnn.py feeds to the CNN at runtime.
    """
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed  = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest white contour — that's your card
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

        # Cut white border symmetrically — 15% from each edge
        margin_x = int(w * 0.15)
        margin_y = int(h * 0.15)
        roi = gray[y+margin_y : y+h-margin_y,
                   x+margin_x : x+w-margin_x]

        if roi.size > 0:
            return cv2.resize(roi, (IMG_SIZE, IMG_SIZE))

    # Fallback — centre crop if no white card found
    h, w = gray.shape[:2]
    side = min(h, w)
    y0   = (h - side) // 2
    x0   = (w - side) // 2
    return cv2.resize(gray[y0:y0+side, x0:x0+side], (IMG_SIZE, IMG_SIZE))

def count_existing(label):
    folder = os.path.join(SAVE_ROOT, label)
    if not os.path.exists(folder):
        return 0
    return len([f for f in os.listdir(folder) if f.endswith('.png')])

def next_index(label):
    """Returns the next safe file index — max existing number + 1, avoiding overwrites."""
    folder = os.path.join(SAVE_ROOT, label)
    if not os.path.exists(folder):
        return 1
    files = [f for f in os.listdir(folder) if f.endswith('.png')]
    if not files:
        return 1
    nums = []
    for f in files:
        try:
            nums.append(int(os.path.splitext(f)[0]))
        except ValueError:
            pass
    return max(nums) + 1 if nums else 1

def save_burst(label, frames):
    folder = os.path.join(SAVE_ROOT, label)
    os.makedirs(folder, exist_ok=True)
    start = next_index(label)
    for i, img in enumerate(frames):
        path = os.path.join(folder, f"{start + i:04d}.png")
        cv2.imwrite(path, img)
    print(f"  ✓ Saved {len(frames)} images for '{label}' "
          f"(total: {count_existing(label) + len(frames)})")

def run():
    camera = select_camera()

    # ── Camera init ────────────────────────────────────────
    if camera == "realsense":
        try:
            import pyrealsense2 as rs
            pipeline = rs.pipeline()
            cfg      = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(cfg)
            def get_frame():
                frames = pipeline.wait_for_frames()
                return np.asanyarray(frames.get_color_frame().get_data())
            print("[Camera] RealSense connected ✓")
        except Exception as e:
            print(f"[Camera] RealSense failed: {e}")
            print("[Camera] Falling back to webcam...")
            camera = "webcam"

    if camera == "webcam":
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[Camera] ERROR: No webcam found.")
            sys.exit(1)
        def get_frame():
            _, frame = cap.read()
            return frame
        print("[Camera] Webcam connected ✓")

    print("\n══════════════════════════════════════════════")
    print("  Block Letter Data Collector")
    print("══════════════════════════════════════════════")
    print(f"  Press a LETTER key to capture 1 image")
    print(f"  Move/tilt the card between EVERY keypress")
    print(f"  Watch the 'Crop Preview' window — that's")
    print(f"  exactly what the CNN will see")
    print("  Press Esc to quit\n")

    current_label    = None
    flash_until      = 0
    counts           = {l: count_existing(l) for l in LABEL_MAP}

    try:
        while True:
            frame   = get_frame()
            display = frame.copy()
            now     = time.time()

            # Flash green on capture
            if now < flash_until:
                overlay = display.copy()
                cv2.rectangle(overlay, (0, 0), (display.shape[1], display.shape[0]),
                              (0, 255, 0), -1)
                display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

            # HUD
            cv2.putText(display,
                        f"Press a key to capture. Last: {current_label or 'none'}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            # Show counts
            y = 60
            for i, label in enumerate(LABEL_MAP):
                col = (0, 255, 0) if counts[label] >= 200 else (0, 180, 255)
                cv2.putText(display, f"{label}:{counts[label]}",
                            (10 + (i % 9) * 70, y + (i // 9) * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

            cv2.imshow("Data Collector", display)

            # ── Preview what will be saved ─────────────────
            # Shows the actual cropped image the CNN will see
            preview = preprocess(frame)
            preview_big = cv2.resize(preview, (256, 256), interpolation=cv2.INTER_NEAREST)
            cv2.putText(preview_big, "CNN INPUT PREVIEW", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Crop Preview", preview_big)
            key = cv2.waitKey(30) & 0xFF

            if key == 27:
                break

            ch = chr(key).upper()
            if ch in LABEL_MAP:
                current_label = ch
                burst = []
                for _ in range(BURST_SIZE):
                    f = get_frame()
                    burst.append(preprocess(f))
                    time.sleep(BURST_DELAY)
                save_burst(current_label, burst)
                counts[current_label] = count_existing(current_label)
                flash_until = time.time() + 0.3

    finally:
        if camera == "realsense":
            pipeline.stop()
        else:
            cap.release()
        cv2.destroyAllWindows()
        print("\n[Done] Total images per label:")
        for label in LABEL_MAP:
            c = count_existing(label)
            print(f"  {label}: {c} images {'✓' if c >= 200 else '⚠ need more'}")

if __name__ == '__main__':
    run()