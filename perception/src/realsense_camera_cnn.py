# Intel RealSense RGB-D Camera - CNN Letter Detection Node
#
# SETUP:
# Terminal 1 (RealSense ROS2 driver):
#   ros2 launch realsense2_camera rs_launch.py enable_depth:=true align_depth.enable:=true
# Terminal 2 (this node):
#   python3 realsense_camera_cnn.py
#
# CONTROLS:
#   SPACEBAR  - Toggle "at position" (enables letter scanning)
#   Q         - Quit
#

# ── COORDINATE FLOW ──────────────────────────────────────────────────────────
#
#  1. RealSense depth + pixel → camera frame (metres)
#       pixel_to_camera_frame() converts block centre pixel + depth reading
#       into (x_m, y_m, z_m) relative to the camera lens.
#       Camera axes: x=right, y=down, z=forward into scene.
#
#  2. Camera frame → world frame (via TF2)
#       At startup this node broadcasts a static TF:
#         gripper_tcp → camera_color_optical_frame
#       using the CAM_MOUNT_* calibration values at the top of this file.
#       TF2 then chains:
#         camera_color_optical_frame → gripper_tcp → ... → base_link → world
#       using the live UR3e joint states from robot_state_publisher.
#       _camera_to_world() performs this lookup and applies the transform.
#
# ── TOPICS SUBSCRIBED ────────────────────────────────────────────────────────
#
#   /camera/camera/color/image_raw          sensor_msgs/Image   — colour frames
#   /camera/camera/aligned_depth_to_color/  sensor_msgs/Image   — aligned depth
#     image_raw
#   /mission/state                          std_msgs/String
#       "SCANNING" → enables CNN scan (same as SPACEBAR on)
#       "IDLE"     → disables scan, publishes gameboard_state snapshot
#
# ── TOPICS PUBLISHED ─────────────────────────────────────────────────────────
#
#   /perception/human_detected    std_msgs/Bool
#       True/False every ~5 Hz. Used by Elijah's safety monitor.
#
#   /perception/status            std_msgs/String
#       "SCANNING" or "IDLE" every ~5 Hz. Used by Elijah's behaviour tree.
#
#   /perception/detections        std_msgs/String  (JSON)
#       Published every frame while scanning. CAMERA FRAME coordinates.
#       Format: {"blocks": [{"letter":"A","conf":94.2,
#                            "x_m":0.04,"y_m":-0.02,"z_m":0.38,
#                            "theta_deg":12.5}]}
#       Subscribers: Kermit (word solver), Elijah (behaviour tree)
#
#   /perception/gameboard_state   hl_control/GameboardState  (LATCHED)
#       Published ONCE per completed scan (on IDLE mission state or spacebar off).
#       WORLD FRAME coordinates — camera→world TF applied.
#       Each detected block becomes a LetterObject:
#         letter    : "A"
#         object_id : "A_object_1"  (unique per letter, e.g. two A's → _1 and _2)
#         pose      : PoseStamped, frame_id="world", position in robot world frame
#       Subscriber: Connor's hl_control_node — triggers pick-and-place planning
#                   once this AND /hl_control/word_request have both arrived.




# ── Mode switch ───────────────────────────────────────────
USE_ROS2 = True   # False = direct pyrealsense2 SDK, True = ROS2 topics

# ── Detection mode ────────────────────────────────────────
# CARD  = paper cards on black foam mat (current setup)
#         uses white brightness threshold, crops inner 60% to remove white border
# BLOCK = wooden blocks on table (robot setup)
#         uses depth gate + adaptive threshold, full bounding box ROI
DETECTION_MODE = "CARD"   # switch to "BLOCK" when on the robot

# ── Detection tuning ──────────────────────────────────────
BLOCK_DEPTH_MIN_M   = 0.10   # EE-mounted camera — blocks are 10-30cm away
BLOCK_DEPTH_MAX_M   = 0.30   # tight gate — only objects at block distance
MIN_BLOCK_AREA      = 16000  # minimum contour area (pixels²)
MAX_BLOCK_AREA      = 240000 # maximum contour area (pixels²)
MAX_BLOCKS          = 5      # max blocks expected in workspace
CNN_CONF_THRESHOLD  = 32.0   # min confidence % to publish a letter
FRAMES_TO_AVERAGE   = 15     # temporal smoothing frames
CARD_BRIGHTNESS     = 180    # brightness threshold for white card detection (0-255)
CARD_MARGIN         = 0.10   # fraction to crop from each edge of card bounding box

ENABLE_HUMAN_DETECTION = False

# ── Out-of-category and scene reprocessing ────────────────
# OUT_OF_CATEGORY: flag blocks that pass depth/area filters
# but CNN confidence is below this on ALL classes — not a letter block
OUT_OF_CATEGORY_CONF_MAX = 20.0   # if max conf across all classes < this, flag as unknown object

# SCENE_CHANGE: if detected block count changes by this many
# between scans, flag the scene as changed and clear gameboard state
SCENE_CHANGE_THRESHOLD = 1
# ─────────────────────────────────────────────────────────

import cv2
import numpy as np
import os
import collections
import json
import statistics
if ENABLE_HUMAN_DETECTION:
    import mediapipe as mp
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import math

# Model path — absolute, works from any working directory
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../outputs/letter_cnn.pt")

# ── Camera intrinsics (D435i @ 640x480) ──────────────────
# These are the factory defaults. Replace with values from
# rs2_intrinsics after running: ros2 topic echo /camera/camera/color/camera_info
# fx, fy = focal lengths in pixels
# cx, cy = principal point (optical centre) in pixels
CAM_FX = 914.617
CAM_FY = 912.751
CAM_CX = 641.396
CAM_CY = 359.803
# TF frame broadcast by the RealSense ROS2 driver for the colour camera
CAMERA_FRAME = "camera_color_optical_frame"

# ── Camera extrinsic calibration (camera mount relative to gripper_tcp) ───
# gripper_tcp is the tool centre point — the midpoint between the gripper fingers.
# These values define where the camera sits relative to that point.
# Edit these to re-calibrate; restart the node to apply. No other code changes needed.
#
# On the UR3e with OnRobot RG2, gripper_tcp z-axis points downward (toward the table).
# Current mount: camera is 5 cm back (toward wrist) and 5 cm up from gripper_tcp.
CAM_MOUNT_X =  0.00   # metres — left/right from gripper_tcp centreline
CAM_MOUNT_Y =  0.05   # metres — upward from gripper_tcp (perpendicular to gripper z)
CAM_MOUNT_Z = -0.05   # metres — back toward wrist along gripper_tcp z-axis

# Rotation: RPY from gripper_tcp frame into camera_color_optical_frame.
# RealSense optical convention: x=right, y=down, z=forward (into scene).
# Adjust if projected world points don't land on the real block positions.
CAM_MOUNT_ROLL  = 0.0  # radians (-90 deg)
CAM_MOUNT_PITCH =  0.0      # radians
CAM_MOUNT_YAW   = 0.0  # radians (-90 deg)
# ─────────────────────────────────────────────────────────


def pixel_to_camera_frame(px, py, depth_m):
    """
    Convert a pixel coordinate + depth to a 3D point in the camera frame (metres).
    This is the raw camera frame — a separate EE-to-robot transform is applied
    downstream by SS2 once camera extrinsics are calibrated.

    Returns (x_m, y_m, z_m) or (None, None, None) if depth is invalid.
    """
    if depth_m <= 0:
        return None, None, None
    x_m = (px - CAM_CX) * depth_m / CAM_FX
    y_m = (py - CAM_CY) * depth_m / CAM_FY
    z_m = depth_m
    return round(x_m, 4), round(y_m, 4), round(z_m, 4)


def estimate_theta(contour):
    """
    Estimate block rotation (degrees) from the minimum area bounding rectangle.
    Returns angle in range [-90, 90] degrees.

    NOTE: This gives the orientation of the block face but does not yet use the
    corner dot to disambiguate 180-degree ambiguity. Dot-based theta will be
    implemented in a later iteration.
    """
    if contour is None or len(contour) < 5:
        return 0.0
    _, _, angle = cv2.minAreaRect(contour)
    # cv2.minAreaRect returns angle in [-90, 0) — normalise to [-90, 90]
    if angle < -45:
        angle += 90
    return round(float(angle), 2)

def detect_dot(frame, x, y, w, h):
    """
    Find the most circular small blob on the card face.
    Searches full inner card — returns raw pixel position or None.
    Smoothing is applied externally on the pixel position itself.
    """
    margin_x = int(w * CARD_MARGIN)
    margin_y = int(h * CARD_MARGIN)
    roi = frame[y+margin_y : y+h-margin_y,
                x+margin_x : x+w-margin_x]
    if roi.size == 0:
        return None

    gray      = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
    kernel    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    cleaned   = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    DOT_MIN_AREA = 8
    DOT_MAX_AREA = 300
    best_circ = 0
    best_M    = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (DOT_MIN_AREA <= area <= DOT_MAX_AREA):
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * math.pi * area / (perimeter ** 2)
        if circularity > best_circ:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            best_circ = circularity
            best_M    = M

    if best_M is None or best_circ < 0.45:
        return None

    dot_px = int(best_M["m10"] / best_M["m00"]) + x + margin_x
    dot_py = int(best_M["m01"] / best_M["m00"]) + y + margin_y
    return (dot_px, dot_py)


def estimate_theta_from_dot(frame, x, y, w, h, dot_pos_buffer=None):
    """
    Calculate true block rotation from dot position.
    Smooths the dot pixel position over N frames before calculating theta.
    This stops theta jumping when the detector flickers between the dot
    and a letter feature — the smoothed position barely moves.
    Dot in top-left = 0 degrees. Falls back to minAreaRect if dot not found.
    """
    cx_px = x + w // 2
    cy_px = y + h // 2

    raw_dot = detect_dot(frame, x, y, w, h)

    if raw_dot is not None and dot_pos_buffer is not None:
        dot_pos_buffer.append(raw_dot)

    # Use smoothed dot position if buffer has enough frames
    if dot_pos_buffer and len(dot_pos_buffer) >= 3:
        # Median of x and y separately — robust to outlier frames
        xs = [p[0] for p in dot_pos_buffer]
        ys = [p[1] for p in dot_pos_buffer]
        dot_px = int(statistics.median(xs))
        dot_py = int(statistics.median(ys))
        dot_found = True
    elif raw_dot is not None:
        dot_px, dot_py = raw_dot
        dot_found = True
    else:
        dot_found = False

    if dot_found:
        dx = dot_px - cx_px
        dy = dot_py - cy_px
        # dot at top-left of card = ~225 deg from centre in image space
        # offset so that position reads as 0 deg
        raw_angle = math.degrees(math.atan2(dy, dx))
        theta = (raw_angle - 225.0 + 360.0) % 360.0
        if theta > 180:
            theta -= 360
        return round(theta, 2), True, (dot_px, dot_py)

    return estimate_theta(None), False, None

# Must match LABEL_MAP in train_letter_cnn.py exactly
LABEL_MAP = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")


# ══════════════════════════════════════════════════════════
# CNN MODEL  — must match train_letter_cnn.py architecture
# ══════════════════════════════════════════════════════════

class LetterCNN(nn.Module):
    def __init__(self, num_classes=36):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),           # 32x32

            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),           # 16x16

            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),           # 8x8
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128 * 8 * 8, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.4),
            nn.Linear(256, num_classes),
        )

    def forward(self, x):
        return self.classifier(self.features(x))


# ══════════════════════════════════════════════════════════
# CNN PREDICTOR
# ══════════════════════════════════════════════════════════

class CNNPredictor:
    def __init__(self, model_path, label_map):
        self.label_map    = label_map
        self.device       = torch.device("cpu")
        self.model        = None
        self.vote_buffers = {}   # always initialised so fallback mode doesn't crash

        if not os.path.exists(model_path):
            print(f"[CNN] WARNING: {model_path} not found — running in fallback mode (shows ?)")
            print(f"      Run train_letter_cnn.py first to generate it.")
            return

        self.model = LetterCNN(num_classes=len(label_map)).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()
        print(f"[CNN] Loaded {model_path}  ({len(label_map)} classes)")
        print(f"[CNN] LABEL_MAP: {''.join(label_map)}")
        print(f"[CNN] NOTE: O/Q confusion detected in training — collect more varied O data if O blocks misread as Q")

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Grayscale(num_output_channels=1),  # handles both grey and BGR input
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize((0.5,), (0.5,)),
        ])

    def _bucket(self, x):
        return x // 64

    def predict(self, roi_bgr, x):
        """Returns (letter, confidence_pct) or (None, conf) if below threshold."""
        if self.model is None or roi_bgr is None or roi_bgr.size == 0:
            return None, 0.0

        try:
            img = self.transform(roi_bgr).unsqueeze(0).to(self.device)
        except Exception:
            return None, 0.0

        with torch.no_grad():
            logits = self.model(img)
            probs  = torch.softmax(logits, dim=1)
            conf, pred = torch.max(probs, 1)

        letter   = self.label_map[pred.item()]
        conf_pct = float(conf.item()) * 100.0

        # Temporal majority vote — smooths out single-frame misreads
        bucket = self._bucket(x)
        if bucket not in self.vote_buffers:
            self.vote_buffers[bucket] = collections.deque(maxlen=FRAMES_TO_AVERAGE)
        self.vote_buffers[bucket].append(letter)
        voted = collections.Counter(self.vote_buffers[bucket]).most_common(1)[0][0]

        if conf_pct >= CNN_CONF_THRESHOLD:
            return voted, conf_pct
        else:
            return None, conf_pct

    def clear_votes(self):
        self.vote_buffers.clear()


# ══════════════════════════════════════════════════════════
# BLOCK DETECTOR
# ══════════════════════════════════════════════════════════

class BlockDetector:
    def find_blocks(self, color_bgr, depth_raw=None):
        gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)

        if DETECTION_MODE == "CARD":
            # ── CARD MODE ─────────────────────────────────
            # White brightness threshold — finds white paper cards on black mat
            # Black mat/background gets rejected regardless of depth
            _, thresh = cv2.threshold(gray, CARD_BRIGHTNESS, 255, cv2.THRESH_BINARY)

        else:
            # ── BLOCK MODE (robot setup) ───────────────────
            # Adaptive threshold + depth gate isolates block faces at known distance
            # Re-tune BLOCK_DEPTH_MIN/MAX_M for your robot camera height
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh  = cv2.adaptiveThreshold(
                blurred, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 11, 2
            )
            if depth_raw is not None:
                depth_m    = depth_raw.astype(np.float32) / 1000.0
                depth_mask = np.logical_and(
                    depth_m >= BLOCK_DEPTH_MIN_M,
                    depth_m <= BLOCK_DEPTH_MAX_M
                ).astype(np.uint8) * 255
                thresh = cv2.bitwise_and(thresh, thresh, mask=depth_mask)

        kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closed   = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # DEBUG — log top contour areas to help tune MIN/MAX_BLOCK_AREA
        areas = sorted([cv2.contourArea(c) for c in contours], reverse=True)[:5]
        #if areas:
            #rint(f"[Debug] Top contour areas: {[int(a) for a in areas]}")

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (MIN_BLOCK_AREA <= area <= MAX_BLOCK_AREA):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if 0.5 <= w / float(h) <= 2.0:
                boxes.append((x, y, w, h, cnt))   # include contour for theta

        boxes.sort(key=lambda b: b[0])
        return boxes[:MAX_BLOCKS]


# ══════════════════════════════════════════════════════════
# ROI CROP — matched to collect_training_data.py pipeline
# ══════════════════════════════════════════════════════════

def extract_roi(frame, x, y, w, h):
    """
    Crop the region fed to the CNN.
    Must match preprocess() in collect_training_data.py exactly:
      greyscale → crop → resize 64x64 → equalizeHist

    CARD mode:  cuts CARD_MARGIN from each edge to remove white border.
    BLOCK mode: uses full bounding box — depth gate already isolated block face.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if DETECTION_MODE == "CARD":
        margin_x = int(w * CARD_MARGIN)
        margin_y = int(h * CARD_MARGIN)
        roi = gray[y+margin_y : y+h-margin_y,
                   x+margin_x : x+w-margin_x]
    else:
        roi = gray[y:y+h, x:x+w]

    if roi.size == 0:
        return None

    resized = cv2.resize(roi, (64, 64))
    return cv2.equalizeHist(resized)  # critical — must match collect_training_data.py


# ══════════════════════════════════════════════════════════
# PERCEPTION
# ══════════════════════════════════════════════════════════

class Perception:
    def __init__(self):
        self.block_detector  = BlockDetector()
        self.cnn             = CNNPredictor(MODEL_PATH, LABEL_MAP)
        self.at_position     = False
        self.last_detections = []
        self.human_detected  = False
        self.theta_buffers   = {}    # x-bucket -> deque of theta values
        self.dot_pos_buffers = {}    # x-bucket -> deque of (dot_px, dot_py) positions

        if ENABLE_HUMAN_DETECTION:
            self.mp_holistic = mp.solutions.holistic
            self.holistic    = self.mp_holistic.Holistic(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.mp_draw = mp.solutions.drawing_utils
        else:
            self.holistic = None
            self.mp_draw  = None


    def process(self, color_bgr, depth_raw=None, depth_colormap=None):
        frame = color_bgr.copy()
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ── Pose / human detection (runs every frame) ─────
        if ENABLE_HUMAN_DETECTION and self.holistic is not None:
            results = self.holistic.process(rgb)
            self.human_detected = results.pose_landmarks is not None
            self.mp_draw.draw_landmarks(
                frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                self.mp_draw.DrawingSpec(color=(0,255,0), thickness=2, circle_radius=2),
                self.mp_draw.DrawingSpec(color=(0,200,0), thickness=2))
            self.mp_draw.draw_landmarks(
                frame, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                self.mp_draw.DrawingSpec(color=(255,100,0), thickness=2, circle_radius=3),
                self.mp_draw.DrawingSpec(color=(255,150,0), thickness=2))
            self.mp_draw.draw_landmarks(
                frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                self.mp_draw.DrawingSpec(color=(0,100,255), thickness=2, circle_radius=3),
                self.mp_draw.DrawingSpec(color=(0,150,255), thickness=2))
            if self.human_detected:
                cv2.putText(frame, "HUMAN DETECTED", (10, frame.shape[0]-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
        else:
            self.human_detected = False


        # ── Depth info per block ──────────────────────────
        depth_info = {}
        if depth_raw is not None:
            for (x, y, w, h, cnt) in self.block_detector.find_blocks(frame, depth_raw):
                roi_d = depth_raw[y:y+h, x:x+w].astype(np.float32) / 1000.0
                valid = roi_d[roi_d > 0]
                if valid.size:
                    depth_info[(x,y,w,h)] = float(np.median(valid))

        # ── CNN detection (only when at position) ─────────
        if self.at_position:
            boxes = self.block_detector.find_blocks(frame, depth_raw)
            detections = []
            for (x, y, w, h, cnt) in boxes:
                roi = extract_roi(frame, x, y, w, h)
                if roi is None:
                    continue
                letter, conf = self.cnn.predict(roi, x)

                # 3D position — centre pixel of block face + median depth
                cx_px = x + w // 2
                cy_px = y + h // 2
                depth_m = depth_info.get((x,y,w,h), 0.0)
                x_m, y_m, z_m = pixel_to_camera_frame(cx_px, cy_px, depth_m)

                # Get or create dot position buffer for this block x-bucket
                bucket = x // 64
                if bucket not in self.dot_pos_buffers:
                    self.dot_pos_buffers[bucket] = collections.deque(maxlen=FRAMES_TO_AVERAGE)

                # Rotation from smoothed dot position — passes buffer so position
                # is smoothed over N frames before theta is calculated
                theta, dot_found, smoothed_dot = estimate_theta_from_dot(
                    frame, x, y, w, h, self.dot_pos_buffers[bucket])

                # If dot found and block too rotated, suppress letter
                # CNN cannot read reliably beyond +-45 degrees
                if dot_found and abs(theta) > 45:
                    letter = None
                    conf = 0.0

                # Out-of-category detection — object passed depth/area filter
                # but CNN is not confident it's any known letter/digit
                out_of_category = (letter is None and conf < OUT_OF_CATEGORY_CONF_MAX)

                detections.append((x, y, w, h, letter, conf, x_m, y_m, z_m, theta, dot_found, smoothed_dot, out_of_category))
            self.last_detections = detections

        # ── Draw detection boxes ──────────────────────────
        for (x, y, w, h, letter, conf, x_m, y_m, z_m, theta, dot_found, smoothed_dot, out_of_category) in self.last_detections:
            # Red box for out-of-category, cyan for normal AT POSITION, grey for MOVING
            if out_of_category:
                box_color = (0, 0, 255)   # red — unknown object
            elif self.at_position:
                box_color = (0, 200, 255) # cyan — reading
            else:
                box_color = (100, 100, 100) # grey — stale

            depth_str = f" {z_m:.2f}m" if z_m else ""

            cv2.rectangle(frame, (x,y), (x+w,y+h), box_color, 2)

            # Draw smoothed dot position — green filled circle
            if smoothed_dot is not None:
                cv2.circle(frame, smoothed_dot, 6, (0, 255, 0), -1)
                cv2.circle(frame, smoothed_dot, 8, (0, 0, 0), 1)

            if out_of_category:
                label = f"UNKNOWN{depth_str} r{theta:.0f}deg"
                cv2.putText(frame, label, (x+4, y+h//2+8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
            elif letter:
                label = f"{letter} {conf:.0f}%{depth_str} r{theta:.0f}deg {'[dot]' if dot_found else '[rect]'}"
                (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
                cv2.rectangle(frame, (x, y-th-bl-6), (x+tw+4, y), box_color, -1)
                cv2.putText(frame, label, (x+2, y-bl-2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 2)
            else:
                low = f"?{depth_str} ({conf:.0f}%) r{theta:.0f}deg {'[dot]' if dot_found else '[rect]'}"
                cv2.putText(frame, low, (x+4, y+h//2+8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)


        # ── HUD ───────────────────────────────────────────
        status_color = (0,255,100) if self.at_position else (0,100,255)
        status_str   = "AT POSITION - READING" if self.at_position else "MOVING - STANDBY"
        cv2.putText(frame, f"{status_str}  [{DETECTION_MODE} MODE]",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        letters = [d[4] for d in self.last_detections if d[4]]
        if letters:
            cv2.putText(frame, "Seen: " + "  ".join(letters),
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,200,255), 2)

        cv2.putText(frame,
                    f"Conf: {CNN_CONF_THRESHOLD:.0f}%  |  Smooth: {FRAMES_TO_AVERAGE}f  |  SPACE: toggle  Q: quit",
                    (10, frame.shape[0]-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)

        # ── Depth map stacked below ───────────────────────
        if depth_colormap is not None:
            depth_resized = cv2.resize(depth_colormap, (frame.shape[1], frame.shape[0]))
            cv2.putText(depth_resized,
                        f"Depth gate: {BLOCK_DEPTH_MIN_M*100:.0f}-{BLOCK_DEPTH_MAX_M*100:.0f}cm",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
            display = np.vstack((frame, depth_resized))
        else:
            display = frame

        return display

    def get_detections_json(self):
        """
        Returns current detections as a JSON string for ROS2 publishing.

        Format:
            {
              "blocks": [
                {
                  "letter":    "A",      // Detected letter (A-Z or 0-9)
                  "conf":      94.2,     // CNN confidence (0-100%)
                  "x_m":       0.0412,   // X position in camera frame (metres)
                  "y_m":      -0.0231,   // Y position in camera frame (metres)
                  "z_m":       0.3820,   // Z depth from camera (metres)
                  "theta_deg": 12.5      // Block rotation around Z axis (degrees)
                                         // From minAreaRect — dot-based refinement TBD
                }
              ]
            }

        NOTE: x_m, y_m, z_m are in the CAMERA frame, not the robot world frame.
        SS2 (Connor) applies the EE-to-robot transform once extrinsics are calibrated.
        If depth is unavailable for a block, x_m/y_m/z_m will be null.
        """
        blocks = []
        letter_counts = {}
        unknown_count = 0
        for (x, y, w, h, letter, conf, x_m, y_m, z_m, theta, dot_found, smoothed_dot, out_of_category) in self.last_detections:
            if letter:
                letter_counts[letter] = letter_counts.get(letter, 0) + 1
                object_id = f"{letter}_object_{letter_counts[letter]}"
            else:
                unknown_count += 1
                object_id = f"unknown_object_{unknown_count}"

            blocks.append({
                "object_id":         object_id,
                "letter":            letter,
                "conf":              round(conf, 1) if conf else 0.0,
                "x_m":               x_m,
                "y_m":               y_m,
                "z_m":               z_m,
                "theta_deg":         theta,
                "rotation_required": theta,
                "dot_found":         dot_found,
                "out_of_category":   out_of_category,
            })
        return json.dumps({"blocks": blocks})

    def toggle_position(self):
        self.at_position = not self.at_position
        self.last_detections = []
        self.cnn.clear_votes()
        self.theta_buffers.clear()
        self.dot_pos_buffers.clear()
        print(f"[Trigger] -> {'AT POSITION' if self.at_position else 'MOVING'}")

    def close(self):
        if self.holistic is not None:
            self.holistic.close()


# ══════════════════════════════════════════════════════════
# MODE A: pyrealsense2 SDK (no ROS2)
# ══════════════════════════════════════════════════════════

def run_sdk():
    import pyrealsense2 as rs
    pipeline  = rs.pipeline()
    config    = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,  30)
    align      = rs.align(rs.stream.color)
    colorizer  = rs.colorizer()
    perception = Perception()
    print("[SDK] Starting. SPACE=toggle, Q=quit.")
    pipeline.start(config)
    try:
        while True:
            frames        = pipeline.wait_for_frames()
            aligned       = align.process(frames)
            color_frame   = aligned.get_color_frame()
            depth_frame   = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            color_image   = np.asanyarray(color_frame.get_data())
            depth_raw     = np.asanyarray(depth_frame.get_data())
            depth_colored = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            display = perception.process(color_image, depth_raw, depth_colored)
            cv2.imshow("RealSense CNN Vision", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                perception.toggle_position()
    finally:
        pipeline.stop()
        perception.close()
        cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════════
# MODE B: ROS2
# ══════════════════════════════════════════════════════════

def run_ros2():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge
    import tf2_ros

    # Latched QoS — Connor's hl_control_node uses TRANSIENT_LOCAL so it receives
    # the message regardless of startup order.
    LATCHED_QOS = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    # Connor's hl_control message types — generated at build time from hl_control/msg/*.msg
    try:
        from hl_control.msg import GameboardState, LetterObject
        HL_CONTROL_AVAILABLE = True
    except ImportError:
        HL_CONTROL_AVAILABLE = False

    class RealSenseCNNNode(Node):
        def __init__(self):
            super().__init__('realsense_cnn_perception')
            self.bridge     = CvBridge()
            self.perception = Perception()

            # ── Camera subscribers ────────────────────────
            self.latest_depth = None
            self._processing  = False
            self.frame_count  = 0

            self.create_subscription(
                Image,
                '/camera/camera/aligned_depth_to_color/image_raw',
                self.depth_callback,
                10)

            self.create_subscription(
                Image,
                '/camera/camera/color/image_raw',
                self.color_callback,
                10)

            # ── Mission state subscriber ──────────────────
            # Elijah's behaviour tree sends "SCANNING" or "IDLE"
            # This replaces the spacebar trigger for robot operation
            self.create_subscription(
                String,
                '/mission/state',
                self.mission_callback,
                10
            )

            # ── TF2 buffer — listens for the full robot TF tree ──
            self.tf_buffer   = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            # ── Static TF: gripper_tcp -> camera_color_optical_frame ──
            # Completes the chain: camera -> gripper_tcp -> ... -> world
            # Uses CAM_MOUNT_* values at the top of this file.
            self._broadcast_camera_static_tf()

            # ── Publishers — existing interface (Kermit / Elijah) ──
            self.pub_human = self.create_publisher(
                Bool, '/perception/human_detected', 10)

            self.pub_status = self.create_publisher(
                String, '/perception/status', 10)

            # Block detections — JSON String, published each scan frame
            # Kermit's word solver and Elijah's safety monitor subscribe to this.
            self.pub_detections = self.create_publisher(
                String, '/perception/detections', 10)

            # CHECK WITH LEAD BEFORE IMPLEMENTING
            # Annotated image — Image, every frame, for Elijah's GUI camera view (CV mode)
            # Shows pose landmarks, bounding boxes, and HUD text
            self.pub_annotated = self.create_publisher(
                Image, '/perception/image_annotated', 10)

            # Scene change — published when block count changes or human detected
            # Elijah's behaviour tree subscribes to trigger re-scan
            self.pub_scene_changed = self.create_publisher(
                String, '/perception/scene_changed', 10)

            # Out of category — published when unknown object detected in workspace
            self.pub_out_of_category = self.create_publisher(
                String, '/perception/out_of_category', 10)

            # Track previous block count for scene change detection
            self._prev_block_count = 0

            # ── Publisher — Connor's HL control interface ──
            # Latched so hl_control_node receives it regardless of startup order.
            # Published once per completed scan, not every frame.
            if HL_CONTROL_AVAILABLE:
                self.pub_gameboard = self.create_publisher(
                    GameboardState, '/perception/gameboard_state', LATCHED_QOS)
                self.get_logger().info(
                    '[SS3] hl_control msgs found — will publish /perception/gameboard_state')
            else:
                self.pub_gameboard = None
                self.get_logger().warn(
                    '[SS3] hl_control package not found — '
                    '/perception/gameboard_state will NOT be published. '
                    'Run: colcon build --packages-select hl_control')

            self.get_logger().info(
                '\nCNN perception node ready.'
                '\n  Subscribing: /camera/camera/color/image_raw'
                '\n               /camera/camera/aligned_depth_to_color/image_raw'
                '\n               /mission/state'
                '\n  Publishing:  /perception/human_detected  (Bool, every frame)'
                '\n               /perception/status          (String, every frame)'
                '\n               /perception/detections      (String JSON, when scanning)'
                '\n               /perception/image_annotated (Image, for GUI CV mode)'
                '\n               /perception/scene_changed   (String JSON, on count change)'
                '\n               /perception/out_of_category (String JSON, unknown objects)'
                '\n               /perception/gameboard_state (GameboardState, latched, on scan complete)'
                '\n  SPACE=manual toggle  Q=quit'
            )

        # ─────────────────────────────────────────────────────────────
        # Mission state callback (Elijah's behaviour tree)
        # ─────────────────────────────────────────────────────────────

        def mission_callback(self, msg):
            """Receive scan trigger from Elijah's behaviour tree."""
            state = msg.data.upper().strip()
            if state == "SCANNING" and not self.perception.at_position:
                self.perception.at_position = True
                self.perception.cnn.clear_votes()
                self.get_logger().info('[Mission] SCANNING -> AT POSITION')
            elif state == "IDLE" and self.perception.at_position:
                self.perception.at_position = False
                # Publish gameboard snapshot to Connor before clearing detections
                self._publish_gameboard_state()
                self.perception.last_detections = []
                self.get_logger().info('[Mission] IDLE -> published gameboard_state -> MOVING')

        # ─────────────────────────────────────────────────────────────
        # Camera callbacks
        # ─────────────────────────────────────────────────────────────

        def depth_callback(self, depth_msg):
            try:
                was_none = self.latest_depth is None
                self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                if was_none:
                    self.get_logger().info('[Camera] First depth frame received')
            except Exception as e:
                self.get_logger().error(f'Depth error: {e}')

        def color_callback(self, color_msg):
            if self._processing:
                return
            self._processing = True
            if self.frame_count == 0:
                self.get_logger().info('[Camera] First colour frame received — starting display')
            try:
                color_bgr = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
                depth_raw = self.latest_depth

                display = self.perception.process(color_bgr, depth_raw, None)

                # ── Publish at reduced rate (every 6th frame = ~5 Hz) ──
                self.frame_count += 1
                if self.frame_count % 6 == 0:
                    human_msg      = Bool()
                    human_msg.data = self.perception.human_detected
                    self.pub_human.publish(human_msg)

                    status_msg      = String()
                    status_msg.data = "SCANNING" if self.perception.at_position else "IDLE"
                    self.pub_status.publish(status_msg)

                # ── Publish detections JSON (Kermit / Elijah) ──────────
                if self.perception.at_position:
                    det_msg      = String()
                    det_msg.data = self.perception.get_detections_json()
                    self.pub_detections.publish(det_msg)

                    # ── Scene change detection ─────────────────────────
                    current_block_count = len(self.perception.last_detections)
                    if abs(current_block_count - self._prev_block_count) >= SCENE_CHANGE_THRESHOLD:
                        scene_msg = String()
                        scene_msg.data = (
                            f'{{"event": "scene_changed", '
                            f'"prev_count": {self._prev_block_count}, '
                            f'"new_count": {current_block_count}}}'
                        )
                        self.pub_scene_changed.publish(scene_msg)
                        self.get_logger().warn(
                            f'[Scene] Block count changed: {self._prev_block_count} -> {current_block_count}')
                    self._prev_block_count = current_block_count

                    # ── Out-of-category publishing ─────────────────────
                    unknown_objects = [
                        d for d in self.perception.last_detections if d[12]
                    ]
                    if unknown_objects:
                        ooc_msg = String()
                        ooc_msg.data = (
                            f'{{"event": "out_of_category", '
                            f'"count": {len(unknown_objects)}}}'
                        )
                        self.pub_out_of_category.publish(ooc_msg)
                        if self.frame_count % 30 == 0:
                            self.get_logger().warn(
                                f'[OOC] {len(unknown_objects)} unknown object(s) in workspace')

                    # Log at ~1 Hz (every 30 frames) to avoid flooding the terminal.
                    # Empty frames are silent — only logs when blocks are detected.
                    if self.frame_count % 30 == 0:
                        detections = self.perception.last_detections
                        if detections:
                            lines = []
                            for (x, y, w, h, letter, conf, x_m, y_m, z_m, theta, dot_found, smoothed_dot, out_of_category) in detections:
                                if not letter:
                                    continue
                                cam_str = f'cam=({x_m:.3f}, {y_m:.3f}, {z_m:.3f})' \
                                          if x_m is not None else 'cam=(no depth)'
                                if x_m is not None:
                                    world = self._camera_to_world(x_m, y_m, z_m)
                                    world_str = f'world=({world[0]:.3f}, {world[1]:.3f}, {world[2]:.3f})' \
                                                if world is not None else 'world=(no TF)'
                                else:
                                    world_str = 'world=(no depth)'
                                lines.append(
                                    f'  {letter} {conf:.0f}%  {cam_str}  {world_str}  theta={theta:.1f}deg'
                                )
                            if lines:
                                self.get_logger().info(
                                    '[Detections]\n' + '\n'.join(lines))

                # CHECK WITH LEAD BEFORE IMPLEMENTING
                # ── Publish annotated image (every frame) ──
                # For Elijah's GUI camera view when in "Computer Vision" mode
                try:
                    ann_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
                    ann_msg.header = color_msg.header
                    self.pub_annotated.publish(ann_msg)
                except Exception as e:
                    self.get_logger().warn(f'annotated publish failed: {e}',
                                           throttle_duration_sec=5.0)

                # ── Display ────────────────────────────────────────────
                display = cv2.resize(display, (960, 540))
                cv2.imshow("RealSense CNN Vision", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()
                elif key == ord(' '):
                    was_at_position = self.perception.at_position
                    self.perception.toggle_position()
                    # Manual toggle: publish gameboard snapshot when leaving AT POSITION
                    if was_at_position and not self.perception.at_position:
                        self._publish_gameboard_state()

            except Exception as e:
                self.get_logger().error(f'Error: {e}')
            finally:
                self._processing = False

        # ─────────────────────────────────────────────────────────────
        # Broadcast static TF: gripper_tcp -> camera_color_optical_frame
        # ─────────────────────────────────────────────────────────────

        def _broadcast_camera_static_tf(self):
            """
            Publish a one-shot static transform from gripper_tcp to
            camera_color_optical_frame using the CAM_MOUNT_* values at the top
            of this file. StaticTransformBroadcaster re-latches it automatically
            so late subscribers always receive it.

            To re-calibrate: edit CAM_MOUNT_X/Y/Z and CAM_MOUNT_ROLL/PITCH/YAW
            at the top of the file and restart the node.
            """
            import math
            from geometry_msgs.msg import TransformStamped

            broadcaster = tf2_ros.StaticTransformBroadcaster(self)

            ts = TransformStamped()
            ts.header.stamp    = self.get_clock().now().to_msg()
            ts.header.frame_id = 'gripper_tcp'   # parent: OnRobot RG2 tool centre point
            ts.child_frame_id  = CAMERA_FRAME    # child:  RealSense colour optical frame

            ts.transform.translation.x = CAM_MOUNT_X
            ts.transform.translation.y = CAM_MOUNT_Y
            ts.transform.translation.z = CAM_MOUNT_Z

            # RPY -> quaternion
            cr = math.cos(CAM_MOUNT_ROLL  / 2.0)
            sr = math.sin(CAM_MOUNT_ROLL  / 2.0)
            cp = math.cos(CAM_MOUNT_PITCH / 2.0)
            sp = math.sin(CAM_MOUNT_PITCH / 2.0)
            cy = math.cos(CAM_MOUNT_YAW   / 2.0)
            sy = math.sin(CAM_MOUNT_YAW   / 2.0)

            ts.transform.rotation.w = cr * cp * cy + sr * sp * sy
            ts.transform.rotation.x = sr * cp * cy - cr * sp * sy
            ts.transform.rotation.y = cr * sp * cy + sr * cp * sy
            ts.transform.rotation.z = cr * cp * sy - sr * sp * cy

            broadcaster.sendTransform(ts)
            self.get_logger().info(
                f'[TF] Static TF broadcast: gripper_tcp -> {CAMERA_FRAME}  '
                f'x={CAM_MOUNT_X:.3f} y={CAM_MOUNT_Y:.3f} z={CAM_MOUNT_Z:.3f}  '
                f'r={CAM_MOUNT_ROLL:.3f} p={CAM_MOUNT_PITCH:.3f} y={CAM_MOUNT_YAW:.3f}')

        # ─────────────────────────────────────────────────────────────
        # Transform a camera-frame point to world frame via TF2
        # ─────────────────────────────────────────────────────────────

        def _camera_to_world(self, x_cam, y_cam, z_cam):
            """
            Look up camera_color_optical_frame -> world through the full UR3e
            TF tree:
              camera_color_optical_frame -> gripper_tcp -> ... -> base_link -> world

            The static TF from _broadcast_camera_static_tf provides the
            camera->gripper_tcp link. The UR3e robot_state_publisher provides
            the rest of the chain automatically.

            Returns (x_w, y_w, z_w) in metres in the world frame,
            or None if the TF tree is not yet complete (e.g. robot driver
            not running yet).
            """
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'world',
                    CAMERA_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f'[TF] camera->world lookup failed: {e}. '
                    f'Is the robot driver running? '
                    f'Falling back to camera frame coordinates.',
                    throttle_duration_sec=5.0)
                return None

            t  = tf_stamped.transform.translation
            r  = tf_stamped.transform.rotation
            qx, qy, qz, qw = r.x, r.y, r.z, r.w

            R = np.array([
                [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
                [  2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),   2*(qy*qz - qx*qw)],
                [  2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
            ])

            p_world = R @ np.array([x_cam, y_cam, z_cam]) + np.array([t.x, t.y, t.z])
            return (
                round(float(p_world[0]), 4),
                round(float(p_world[1]), 4),
                round(float(p_world[2]), 4),
            )

        # ─────────────────────────────────────────────────────────────
        # Publish gameboard_state (Connor's hl_control interface)
        # Triggered on: SCANNING->IDLE mission transition AND spacebar toggle-off
        # ─────────────────────────────────────────────────────────────

        def _publish_gameboard_state(self):
            """
            Build and latch-publish hl_control/GameboardState on
            /perception/gameboard_state.

            Each confident detection becomes a LetterObject:
              letter    — upper-case single character
              object_id — "{LETTER}_object_{n}" unique per letter per scan
              pose      — PoseStamped in world frame (TF2 transform applied)
                          Falls back to camera frame if TF unavailable.
            """
            if self.pub_gameboard is None:
                return

            if not self.perception.last_detections:
                self.get_logger().info('[gameboard_state] No detections — skipping publish.')
                return

            try:
                from hl_control.msg import GameboardState, LetterObject
            except ImportError:
                return

            board  = GameboardState()
            now    = self.get_clock().now().to_msg()
            counts = {}   # per-letter counter for unique object IDs

            for (x, y, w, h, letter, conf, x_m, y_m, z_m, theta, dot_found, smoothed_dot, out_of_category) in self.perception.last_detections:
                if not letter:
                    continue
                if x_m is None:
                    self.get_logger().warn(
                        f'[gameboard_state] {letter} has no depth — skipping.')
                    continue

                counts[letter] = counts.get(letter, 0) + 1
                object_id = f'{letter}_object_{counts[letter]}'

                world_coords = self._camera_to_world(x_m, y_m, z_m)

                ps              = PoseStamped()
                ps.header.stamp = now

                if world_coords is not None:
                    wx, wy, wz = world_coords
                    ps.header.frame_id    = 'world'
                    ps.pose.position.x    = wx
                    ps.pose.position.y    = wy
                    ps.pose.position.z    = wz
                    # Yaw from minAreaRect theta — rotation around Z axis.
                    # Gives Connor gripper yaw to align with block orientation.
                    theta_rad = math.radians(theta)
                    ps.pose.orientation.w = math.cos(theta_rad / 2.0)
                    ps.pose.orientation.x = 0.0
                    ps.pose.orientation.y = 0.0
                    ps.pose.orientation.z = math.sin(theta_rad / 2.0)



                else:
                    # TF not yet available — fall back to camera frame
                    ps.header.frame_id    = CAMERA_FRAME
                    ps.pose.position.x    = x_m
                    ps.pose.position.y    = y_m
                    ps.pose.position.z    = z_m
                    ps.pose.orientation.w = 1.0
                    self.get_logger().warn(
                        f'[gameboard_state] {object_id} published in camera frame (no TF).')

                lo           = LetterObject()
                lo.letter    = letter
                lo.object_id = object_id
                lo.pose      = ps
                board.letters.append(lo)

                self.get_logger().info(
                    f'[gameboard_state] {object_id}: '
                    f'frame={ps.header.frame_id} '
                    f'x={ps.pose.position.x:.4f} '
                    f'y={ps.pose.position.y:.4f} '
                    f'z={ps.pose.position.z:.4f}')

            if not board.letters:
                self.get_logger().warn('[gameboard_state] No valid blocks to publish.')
                return

            self.pub_gameboard.publish(board)
            self.get_logger().info(
                f'[gameboard_state] Published {len(board.letters)} block(s) '
                f'on /perception/gameboard_state (latched).')

        def destroy_node(self):
            self.perception.close()
            cv2.destroyAllWindows()
            super().destroy_node()

    rclpy.init()
    node = RealSenseCNNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ══════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════

if __name__ == '__main__':
    if USE_ROS2:
        run_ros2()
    else:
        run_sdk()
