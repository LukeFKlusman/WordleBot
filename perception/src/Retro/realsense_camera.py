# Intel RealSense RGB-D Camera - Smart Perception Node
# With block letter detection, depth gating, and spacebar position trigger
#
# SETUP:
# Terminal 1 (RealSense ROS2 driver - if USE_ROS2=True):
#   ros2 launch realsense2_camera rs_launch.py enable_depth:=true align_depth.enable:=true
#
# Terminal 2 (this node):
#   python3 realsense_camera.py
#
# CONTROLS:
#   SPACEBAR  - Toggle "at position" trigger (enables/disables letter detection)
#   Q         - Quit
#
# DEPENDENCIES:
#   pip install pyrealsense2 mediapipe pytesseract opencv-python numpy
#   sudo apt install tesseract-ocr

USE_ROS2 = True  # <-- Toggle: False = pyrealsense2 SDK, True = ROS2 topics

# ── Detection tuning ────────────────────c──────────────────────
BLOCK_DEPTH_MIN_M   = 0.10   # Ignore objects closer than 10cm
BLOCK_DEPTH_MAX_M   = 0.80   # Only detect blocks within 80cm
MIN_BLOCK_AREA      = 1500   # Min contour area (px²) to be considered a block
MAX_BLOCK_AREA      = 80000  # Max contour area - filters out large background blobs
MAX_BLOCKS          = 8      # Cap detections to avoid noise
OCR_CONFIDENCE_MIN  = 50     # Minimum Tesseract confidence (0-100) to display result
# ─────────────────────────────────────────────────────────────

import cv2
import mediapipe as mp
import pytesseract
import numpy as np

# ─────────────────────────────────────────────────────────────
# BLOCK DETECTOR
# ─────────────────────────────────────────────────────────────

class BlockDetector:
    """
    Finds rectangular block-like contours in the colour frame,
    optionally gated by a depth mask so only blocks within
    BLOCK_DEPTH_MIN_M..BLOCK_DEPTH_MAX_M are considered.
    Returns a list of (x, y, w, h) bounding boxes.
    """
    def find_blocks(self, color_bgr, depth_raw=None):
        gray    = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh  = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 11, 2
        )

        # Gate by depth if available
        if depth_raw is not None:
            depth_m    = depth_raw.astype(np.float32) / 1000.0  # mm -> m
            depth_mask = np.logical_and(
                depth_m >= BLOCK_DEPTH_MIN_M,
                depth_m <= BLOCK_DEPTH_MAX_M
            ).astype(np.uint8) * 255
            thresh = cv2.bitwise_and(thresh, thresh, mask=depth_mask)

        # Morphological closing to join block edges
        kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closed  = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (MIN_BLOCK_AREA <= area <= MAX_BLOCK_AREA):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            # Prefer roughly square-ish shapes (blocks) - aspect ratio filter
            aspect = w / float(h)
            if 0.3 <= aspect <= 3.0:
                boxes.append((x, y, w, h))

        # Sort left-to-right, cap count
        boxes.sort(key=lambda b: b[0])
        return boxes[:MAX_BLOCKS]


# ─────────────────────────────────────────────────────────────
# OCR  (PSM 10 - single character per crop)
# ─────────────────────────────────────────────────────────────

def read_letter(roi_bgr):
    """
    Returns (letter, confidence) for the best single-character result,
    or (None, 0) if nothing confident enough was found.
    """
    gray    = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    # Upscale small ROIs - Tesseract struggles below ~64px
    h, w    = gray.shape
    scale   = max(1, 64 // min(h, w))
    if scale > 1:
        gray = cv2.resize(gray, (w * scale, h * scale), interpolation=cv2.INTER_CUBIC)

    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    thresh  = cv2.adaptiveThreshold(
        blurred, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2
    )
    # Add white border - helps Tesseract with edge characters
    padded  = cv2.copyMakeBorder(thresh, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=255)

    data = pytesseract.image_to_data(
        padded,
        config='--psm 10 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789',
        output_type=pytesseract.Output.DICT
    )

    best_letter, best_conf = None, 0
    for text, conf in zip(data['text'], data['conf']):
        text = text.strip()
        try:
            conf = int(conf)
        except (ValueError, TypeError):
            continue
        if text and conf >= OCR_CONFIDENCE_MIN and conf > best_conf:
            best_letter = text[0].upper()
            best_conf   = conf

    return best_letter, best_conf


# ─────────────────────────────────────────────────────────────
# SHARED PERCEPTION LOGIC
# ─────────────────────────────────────────────────────────────

class Perception:
    def __init__(self):
        self.mp_holistic    = mp.solutions.holistic
        self.holistic       = self.mp_holistic.Holistic(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw        = mp.solutions.drawing_utils
        self.block_detector = BlockDetector()

        # Position trigger state (toggled by spacebar)
        self.at_position    = False

        # Persist last OCR results so they stay on screen between reads
        self.last_detections = []   # list of (x, y, w, h, letter, conf)

    def process(self, color_bgr, depth_raw=None, depth_colormap=None):
        frame = color_bgr.copy()
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ── 1. Human / pose detection (always on) ────────────
        results = self.holistic.process(rgb)
        self.mp_draw.draw_landmarks(
            frame, results.pose_landmarks,
            self.mp_holistic.POSE_CONNECTIONS,
            self.mp_draw.DrawingSpec(color=(0, 255, 0),   thickness=2, circle_radius=2),
            self.mp_draw.DrawingSpec(color=(0, 200, 0),   thickness=2)
        )
        self.mp_draw.draw_landmarks(
            frame, results.left_hand_landmarks,
            self.mp_holistic.HAND_CONNECTIONS,
            self.mp_draw.DrawingSpec(color=(255, 100, 0), thickness=2, circle_radius=3),
            self.mp_draw.DrawingSpec(color=(255, 150, 0), thickness=2)
        )
        self.mp_draw.draw_landmarks(
            frame, results.right_hand_landmarks,
            self.mp_holistic.HAND_CONNECTIONS,
            self.mp_draw.DrawingSpec(color=(0, 100, 255), thickness=2, circle_radius=3),
            self.mp_draw.DrawingSpec(color=(0, 150, 255), thickness=2)
        )
        if results.pose_landmarks:
            cv2.putText(frame, "HUMAN DETECTED",
                        (10, frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        # ── 2. Block letter detection (only when at position) ─
        if self.at_position:
            boxes = self.block_detector.find_blocks(frame, depth_raw)
            detections = []
            for (x, y, w, h) in boxes:
                # Slight inset to avoid reading the block edge
                pad  = max(4, min(w, h) // 8)
                roi  = frame[y+pad : y+h-pad, x+pad : x+w-pad]
                if roi.size == 0:
                    continue
                letter, conf = read_letter(roi)
                detections.append((x, y, w, h, letter, conf))
            self.last_detections = detections

        # ── 3. Draw bounding boxes + letters ─────────────────
        for (x, y, w, h, letter, conf) in self.last_detections:
            box_color = (0, 200, 255) if self.at_position else (100, 100, 100)
            cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
            if letter:
                label = f"{letter} ({conf}%)"
                (tw, th), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                # Label background pill
                cv2.rectangle(frame,
                              (x, y - th - baseline - 6),
                              (x + tw + 4, y),
                              box_color, -1)
                cv2.putText(frame, label,
                            (x + 2, y - baseline - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            else:
                cv2.putText(frame, "?", (x + w//2 - 8, y + h//2 + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, box_color, 2)

        # ── 4. HUD overlays ───────────────────────────────────
        status_color = (0, 255, 100) if self.at_position else (0, 100, 255)
        status_text  = "AT POSITION - READING" if self.at_position else "MOVING - STANDBY"
        cv2.putText(frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(frame, "SPACE: toggle position  |  Q: quit",
                    (10, frame.shape[0] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Letter summary bar
        if self.last_detections:
            letters = [d[4] for d in self.last_detections if d[4]]
            if letters:
                summary = "Blocks: " + "  ".join(letters)
                cv2.putText(frame, summary, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)

        # ── 5. Side-by-side with depth ────────────────────────
        if depth_colormap is not None:
            depth_resized = cv2.resize(depth_colormap, (frame.shape[1], frame.shape[0]))
            cv2.putText(depth_resized,
                        f"Gate: {BLOCK_DEPTH_MIN_M*100:.0f}-{BLOCK_DEPTH_MAX_M*100:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            display = np.hstack((frame, depth_resized))
            cv2.putText(display, "RGB + Detections",
                        (10, display.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
            cv2.putText(display, "Depth Map",
                        (frame.shape[1] + 10, display.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        else:
            display = frame

        return display

    def toggle_position(self):
        self.at_position     = not self.at_position
        self.last_detections = []  # Clear stale results on toggle
        state = "AT POSITION" if self.at_position else "MOVING"
        print(f"[Trigger] Position state -> {state}")

    def close(self):
        self.holistic.close()


# ─────────────────────────────────────────────────────────────
# MODE A: DIRECT pyrealsense2 SDK  (USE_ROS2 = False)
# ─────────────────────────────────────────────────────────────

def run_sdk():
    import pyrealsense2 as rs

    pipeline  = rs.pipeline()
    config    = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

    align      = rs.align(rs.stream.color)
    colorizer  = rs.colorizer()
    perception = Perception()

    print("[RealSense SDK] Starting... SPACE to toggle position, Q to quit.")
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
            depth_raw     = np.asanyarray(depth_frame.get_data())          # uint16 mm
            depth_colored = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            display = perception.process(color_image, depth_raw, depth_colored)
            cv2.imshow("RealSense Smart Vision", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                perception.toggle_position()
    finally:
        pipeline.stop()
        perception.close()
        cv2.destroyAllWindows()
        print("[RealSense SDK] Stopped.")


# ─────────────────────────────────────────────────────────────
# MODE B: ROS2 SUBSCRIBER  (USE_ROS2 = True)
# ─────────────────────────────────────────────────────────────

def run_ros2():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from message_filters import ApproximateTimeSynchronizer, Subscriber

    class RealSensePerceptionNode(Node):
        def __init__(self):
            super().__init__('realsense_perception')
            self.bridge     = CvBridge()
            self.perception = Perception()

            color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
            depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
            self.sync = ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=10, slop=0.05)
            self.sync.registerCallback(self.callback)
            self.get_logger().info('RealSense perception node ready. SPACE to toggle, Q to quit.')

        def callback(self, color_msg, depth_msg):
            try:
                color_bgr     = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
                depth_raw     = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                depth_norm    = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(
                    depth_norm.astype(np.uint8), cv2.COLORMAP_JET)

                display = self.perception.process(color_bgr, depth_raw, depth_colored)
                cv2.imshow("RealSense Smart Vision", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()
                elif key == ord(' '):
                    self.perception.toggle_position()

            except Exception as e:
                self.get_logger().error(f'Perception error: {e}')

        def destroy_node(self):
            self.perception.close()
            cv2.destroyAllWindows()
            super().destroy_node()

    rclpy.init()
    node = RealSensePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    if USE_ROS2:
        run_ros2()
    else:
        run_sdk()