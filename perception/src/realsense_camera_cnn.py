# Intel RealSense RGB-D Camera - CNN Letter Detection Node
#
# SETUP:
# Terminal 1 (RealSense ROS2 driver):
#   ros2 launch realsense2_camera rs_launch.py \
#     enable_depth:=true align_depth.enable:=true \
#     depth_module.depth_profile:=640x480x30 \
#     rgb_camera.color_profile:=640x480x30
# Terminal 2 (this node):
#   python3 realsense_camera_cnn.py
#
# CONTROLS:
#   SPACEBAR  - Toggle "at position" (enables letter scanning)
#   Q         - Quit
#
# ROS2 TOPICS PUBLISHED:
#   /perception/human_detected    std_msgs/Bool    — streams every frame (-> SS1 Elijah)
#   /perception/status            std_msgs/String  — "SCANNING" or "IDLE", every frame (-> SS1)
#   /perception/detections        std_msgs/String  — JSON block list with 4DoF pose (-> SS2 Connor, SS4 Kermit)
#   /perception/block_poses       geometry_msgs/PoseArray — 3D block centres for motion planning (-> SS2 Connor)
#   /perception/letters           std_msgs/String  — comma-separated letter list e.g. "A,B,C" (-> SS4 Kermit)
#
# ROS2 TOPICS SUBSCRIBED:
#   /mission/state                std_msgs/String  — "SCANNING" or "IDLE" from SS1 Elijah
#
# PERFORMANCE NOTES (EliteBook 640 / D435i):
#   - MediaPipe runs every POSE_EVERY_N frames (default 10) to save CPU
#   - Display is resized to DISPLAY_W x DISPLAY_H before imshow
#   - CNN runs every CNN_EVERY_N frames when scanning (default 2)
#   - QoS set to BEST_EFFORT to match RealSense driver defaults

# ── Mode switch ───────────────────────────────────────────
USE_ROS2 = True   # False = direct pyrealsense2 SDK, True = ROS2 topics

# ── Detection mode ────────────────────────────────────────
# CARD  = paper cards on black foam mat (current setup)
# BLOCK = wooden blocks on table (robot setup)
DETECTION_MODE = "CARD"

# ── Detection tuning ──────────────────────────────────────
BLOCK_DEPTH_MIN_M   = 0.10
BLOCK_DEPTH_MAX_M   = 0.30
MIN_BLOCK_AREA      = 5000
MAX_BLOCK_AREA      = 200000
MAX_BLOCKS          = 5
CNN_CONF_THRESHOLD  = 32.0
FRAMES_TO_AVERAGE   = 15
CARD_BRIGHTNESS     = 180
CARD_MARGIN         = 0.10

# ── Performance tuning ────────────────────────────────────
POSE_EVERY_N  = 10    # only run MediaPipe every N frames — saves ~60% CPU
CNN_EVERY_N   = 2     # only run CNN every N frames when scanning
DISPLAY_W     = 640   # resize display before imshow
DISPLAY_H     = 360
# ─────────────────────────────────────────────────────────

import cv2
import numpy as np
import os
import collections
import json
import mediapipe as mp
import torch
import torch.nn as nn
import torchvision.transforms as transforms

# Model path — absolute, works from any working directory
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../outputs/letter_cnn.pt")

# ── Camera intrinsics (D435i @ 640x480) ──────────────────
CAM_FX = 615.0
CAM_FY = 615.0
CAM_CX = 320.0
CAM_CY = 240.0
# ─────────────────────────────────────────────────────────


def pixel_to_camera_frame(px, py, depth_m):
    if depth_m <= 0:
        return None, None, None
    x_m = (px - CAM_CX) * depth_m / CAM_FX
    y_m = (py - CAM_CY) * depth_m / CAM_FY
    z_m = depth_m
    return round(x_m, 4), round(y_m, 4), round(z_m, 4)


def estimate_theta(contour):
    if contour is None or len(contour) < 5:
        return 0.0
    _, _, angle = cv2.minAreaRect(contour)
    if angle < -45:
        angle += 90
    return round(float(angle), 2)


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
            nn.MaxPool2d(2),

            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
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
        self.vote_buffers = {}

        if not os.path.exists(model_path):
            print(f"[CNN] WARNING: {model_path} not found — running in fallback mode (shows ?)")
            return

        self.model = LetterCNN(num_classes=len(label_map)).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()
        print(f"[CNN] Loaded {model_path}  ({len(label_map)} classes)")
        print(f"[CNN] LABEL_MAP: {''.join(label_map)}")
        print(f"[CNN] NOTE: O/Q confusion — collect more varied O data if O blocks misread as Q")

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Grayscale(num_output_channels=1),
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize((0.5,), (0.5,)),
        ])

    def _bucket(self, x):
        return x // 64

    def predict(self, roi_bgr, x):
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
            _, thresh = cv2.threshold(gray, CARD_BRIGHTNESS, 255, cv2.THRESH_BINARY)
        else:
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

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (MIN_BLOCK_AREA <= area <= MAX_BLOCK_AREA):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if 0.5 <= w / float(h) <= 2.0:
                boxes.append((x, y, w, h, cnt))

        boxes.sort(key=lambda b: b[0])
        return boxes[:MAX_BLOCKS]


# ══════════════════════════════════════════════════════════
# ROI CROP
# ══════════════════════════════════════════════════════════

def extract_roi(frame, x, y, w, h):
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
    return cv2.equalizeHist(resized)


# ══════════════════════════════════════════════════════════
# PERCEPTION
# ══════════════════════════════════════════════════════════

class Perception:
    def __init__(self):
        # MediaPipe — initialised once, only called every POSE_EVERY_N frames
        self.mp_holistic = mp.solutions.holistic
        self.holistic    = self.mp_holistic.Holistic(
            model_complexity=0,              # lightest model
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw         = mp.solutions.drawing_utils
        self.block_detector  = BlockDetector()
        self.cnn             = CNNPredictor(MODEL_PATH, LABEL_MAP)
        self.at_position     = False
        self.last_detections = []   # (x, y, w, h, letter, conf, x_m, y_m, z_m, theta)
        self.human_detected  = False

        # Frame counters for throttling
        self.frame_count     = 0
        self.last_pose_result = None   # cached MediaPipe result between runs

    def process(self, color_bgr, depth_raw=None, depth_colormap=None):
        frame = color_bgr.copy()
        self.frame_count += 1

        # ── Pose / human detection — throttled ────────────
        # Only run MediaPipe every POSE_EVERY_N frames to save CPU.
        # Cached result is used for drawing in between.
        if self.frame_count % POSE_EVERY_N == 0:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.last_pose_result = self.holistic.process(rgb)
            self.human_detected = (
                self.last_pose_result.pose_landmarks is not None
            )

        results = self.last_pose_result
        if results is not None:
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

        # ── Depth info per block ──────────────────────────
        depth_info = {}
        if depth_raw is not None:
            for (x, y, w, h, cnt) in self.block_detector.find_blocks(frame, depth_raw):
                roi_d = depth_raw[y:y+h, x:x+w].astype(np.float32) / 1000.0
                valid = roi_d[roi_d > 0]
                if valid.size:
                    depth_info[(x,y,w,h)] = float(np.median(valid))

        # ── CNN detection — throttled when scanning ────────
        # Only re-run CNN every CNN_EVERY_N frames.
        # last_detections holds the most recent result in between.
        if self.at_position and self.frame_count % CNN_EVERY_N == 0:
            boxes = self.block_detector.find_blocks(frame, depth_raw)
            detections = []
            for (x, y, w, h, cnt) in boxes:
                roi = extract_roi(frame, x, y, w, h)
                if roi is None:
                    continue
                letter, conf = self.cnn.predict(roi, x)

                cx_px = x + w // 2
                cy_px = y + h // 2
                depth_m = depth_info.get((x,y,w,h), 0.0)
                x_m, y_m, z_m = pixel_to_camera_frame(cx_px, cy_px, depth_m)
                theta = estimate_theta(cnt)

                detections.append((x, y, w, h, letter, conf, x_m, y_m, z_m, theta))
            self.last_detections = detections

        # ── Draw detection boxes ──────────────────────────
        for (x, y, w, h, letter, conf, x_m, y_m, z_m, theta) in self.last_detections:
            box_color = (0, 200, 255) if self.at_position else (100, 100, 100)
            depth_str = f" {z_m:.2f}m" if z_m else ""

            cv2.rectangle(frame, (x,y), (x+w,y+h), box_color, 2)

            if letter:
                label = f"{letter} {conf:.0f}%{depth_str} r{theta:.0f}deg"
                (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
                cv2.rectangle(frame, (x, y-th-bl-6), (x+tw+4, y), box_color, -1)
                cv2.putText(frame, label, (x+2, y-bl-2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 2)
            else:
                low = f"?{depth_str} ({conf:.0f}%)"
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

        # ── Resize display to reduce GPU/render load ──────
        display = cv2.resize(display, (DISPLAY_W, DISPLAY_H))

        return display

    def get_detections_json(self):
        blocks = []
        for (x, y, w, h, letter, conf, x_m, y_m, z_m, theta) in self.last_detections:
            if letter:
                blocks.append({
                    "letter":    letter,
                    "conf":      round(conf, 1),
                    "x_m":       x_m,
                    "y_m":       y_m,
                    "z_m":       z_m,
                    "theta_deg": theta,
                })
        return json.dumps({"blocks": blocks})

    def toggle_position(self):
        self.at_position = not self.at_position
        self.last_detections = []
        self.cnn.clear_votes()
        print(f"[Trigger] -> {'AT POSITION' if self.at_position else 'MOVING'}")

    def close(self):
        self.holistic.close()


# ══════════════════════════════════════════════════════════
# MODE A: pyrealsense2 SDK (no ROS2)
# ══════════════════════════════════════════════════════════

def run_sdk():
    import pyrealsense2 as rs
    pipeline  = rs.pipeline()
    config    = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
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
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseArray, Pose
    from cv_bridge import CvBridge
    from message_filters import ApproximateTimeSynchronizer, Subscriber

    # BEST_EFFORT matches the RealSense driver's default QoS.
    # Without this the synchroniser never receives depth frames,
    # causing the "sequence size exceeds remaining buffer" error.
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    class RealSenseCNNNode(Node):
        def __init__(self):
            super().__init__('realsense_cnn_perception')
            self.bridge     = CvBridge()
            self.perception = Perception()

            # ── Camera subscribers (BEST_EFFORT QoS) ──────
            color_sub = Subscriber(
                self, Image,
                '/camera/camera/color/image_raw',
                qos_profile=sensor_qos)
            depth_sub = Subscriber(
                self, Image,
                '/camera/camera/aligned_depth_to_color/image_raw',
                qos_profile=sensor_qos)
            self.sync = ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=5, slop=0.05)
            self.sync.registerCallback(self.camera_callback)

            # ── Mission state subscriber ──────────────────
            self.create_subscription(
                String,
                '/mission/state',
                self.mission_callback,
                10
            )

            # ── Publishers ────────────────────────────────
            self.pub_human = self.create_publisher(
                Bool, '/perception/human_detected', 10)
            self.pub_status = self.create_publisher(
                String, '/perception/status', 10)
            self.pub_detections = self.create_publisher(
                String, '/perception/detections', 10)
            self.pub_poses = self.create_publisher(
                PoseArray, '/perception/block_poses', 10)
            self.pub_letters = self.create_publisher(
                String, '/perception/letters', 10)

            self.get_logger().info(
                '\nCNN perception node ready.'
                '\n  Subscribing: /camera/camera/color/image_raw'
                '\n               /camera/camera/aligned_depth_to_color/image_raw'
                '\n               /mission/state          (from SS1 Elijah)'
                '\n  Publishing:  /perception/human_detected  (Bool, every frame -> SS1)'
                '\n               /perception/status          (String, every frame -> SS1)'
                '\n               /perception/detections      (JSON, when scanning -> SS2 + SS4)'
                '\n               /perception/block_poses     (PoseArray, when scanning -> SS2)'
                '\n               /perception/letters         (String, when scanning -> SS4)'
                f'\n  Perf: MediaPipe every {POSE_EVERY_N} frames | '
                f'CNN every {CNN_EVERY_N} frames | display {DISPLAY_W}x{DISPLAY_H}'
                '\n  SPACE=manual toggle  Q=quit'
            )

        def mission_callback(self, msg):
            state = msg.data.upper().strip()
            if state == "SCANNING" and not self.perception.at_position:
                self.perception.at_position = True
                self.perception.cnn.clear_votes()
                self.get_logger().info('[Mission] SCANNING -> AT POSITION')
            elif state == "IDLE" and self.perception.at_position:
                self.perception.at_position = False
                self.perception.last_detections = []
                self.get_logger().info('[Mission] IDLE -> MOVING')

        def camera_callback(self, color_msg, depth_msg):
            try:
                color_bgr     = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
                depth_raw     = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                depth_norm    = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)

                display = self.perception.process(color_bgr, depth_raw, depth_colored)

                # ── Publish human detected (every frame) ──
                human_msg      = Bool()
                human_msg.data = self.perception.human_detected
                self.pub_human.publish(human_msg)

                # ── Publish status (every frame) ───────────
                status_msg      = String()
                status_msg.data = "SCANNING" if self.perception.at_position else "IDLE"
                self.pub_status.publish(status_msg)

                # ── Publish detections (when scanning) ─────
                if self.perception.at_position:
                    det_msg      = String()
                    det_msg.data = self.perception.get_detections_json()
                    self.pub_detections.publish(det_msg)
                    self.get_logger().info(f'[Detections] {det_msg.data}')

                    # ── Block poses for Connor (SS2) ───────
                    pose_array = PoseArray()
                    pose_array.header.stamp    = self.get_clock().now().to_msg()
                    pose_array.header.frame_id = 'camera_color_optical_frame'
                    for (_, _, _, _, letter, conf, x_m, y_m, z_m, _theta) in self.perception.last_detections:
                        if letter is None or x_m is None:
                            continue
                        p = Pose()
                        p.position.x    = x_m
                        p.position.y    = y_m
                        p.position.z    = z_m + 0.04  # top face -> block centre
                        p.orientation.w = 1.0
                        pose_array.poses.append(p)
                    self.pub_poses.publish(pose_array)

                    # ── Letters for Kermit (SS4) ───────────
                    letters = [d[4] for d in self.perception.last_detections if d[4]]
                    if letters:
                        letters_msg      = String()
                        letters_msg.data = ','.join(letters)
                        self.pub_letters.publish(letters_msg)
                        self.get_logger().info(f'[Letters] {letters_msg.data}')

                # ── Display ───────────────────────────────
                cv2.imshow("RealSense CNN Vision", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()
                elif key == ord(' '):
                    self.perception.toggle_position()

            except Exception as e:
                self.get_logger().error(f'Error: {e}')

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
        # Guard against double-shutdown (rclpy.shutdown() inside Q handler
        # already calls this before KeyboardInterrupt reaches here)
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════

if __name__ == '__main__':
    if USE_ROS2:
        run_ros2()
    else:
        run_sdk()