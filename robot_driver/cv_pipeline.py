import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="google.protobuf")

import os
import time
import threading
from typing import Dict, Optional, Callable
import cv2
import mediapipe as mp
import numpy as np

# Quiet some verbose logs from underlying CV libs (e.g., MediaPipe)
os.environ.setdefault("GLOG_minloglevel", "2")
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['MEDIAPIPE_DISABLE_GPU'] = '1'

# ===== DEFAULT CV PIPELINE SETTINGS =====

CAM_INDEX = 4                      # webcam index
CONF_THRESHOLD = 0.70             # minimum visibility before computing angles
TARGET_FPS = 30
TARGET_DT = 1.0 / TARGET_FPS

# These are the angles your MediaPipe pipeline computes.
# Using consistent keys so GUI & collision module know what to expect.
ANGLE_KEYS = {
    "left_elbow": "left_elbow",
    "left_shoulder": "left_shoulder",
    "left_wrist": "left_wrist",
    "right_elbow": "right_elbow",
    "right_shoulder": "right_shoulder",
    "right_wrist": "right_wrist",
}




def calculate_angle(a, b, c):
    """Compute angle ABC (in degrees) given three points."""
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)

    ba = a - b
    bc = c - b

    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    cosang = np.clip(cosang, -1.0, 1.0)   # numerical safety
    ang = np.arccos(cosang)
    return np.degrees(ang)




# ==============================
#      CV PIPELINE CLASS
# ==============================
class CVPipeline:
    """
    Threaded CV pipeline:
      - opens webcam
      - runs MediaPipe Pose
      - extracts joint angles (later steps)
      - draws skeleton (later steps)
      - stores latest image + angles
      - GUI polls data
    """

    def __init__(self, cam_index: int = CAM_INDEX, cv_to_node=None):
        self.cam_index = cam_index
        self.running   = False

        # Buffers updated by worker
        self.latest_frame  = None
        self.latest_angles = {}

        # Thread handle
        self._thread = None

        # MediaPipe + Webcam fields (initialized on start)
        self._mp_pose = None
        self._pose    = None
        self._mp_draw = None
        self._cap     = None

        # for smoothing
        self.prev_angles = {}

        # Mapping supplied externally by the GUI
        self.cv_to_node = cv_to_node


    # ---------------- PUBLIC API ----------------
    def start(self):
        if self.running:
            return

        self.running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False

    def get_latest_frame(self):
        return self.latest_frame

    def get_latest_angles(self):
        return dict(self.latest_angles)

    # ---------------- WORKER LOOP ----------------
    def _worker_loop(self):
        """Background thread: init CV stack, then run frame loop."""
        # --- Initialize webcam ---
        print("[CV] Worker started\n")

        self._cap = cv2.VideoCapture(self.cam_index)
        print("[CV] Opened camera:", self.cam_index, self._cap.isOpened())

        if not self._cap.isOpened():
            print("[CV] ERROR: Failed to open webcam.")
            self.running = False
            return

        # --- Initialize MediaPipe Pose ---
        self._mp_pose = mp.solutions.pose
        self._pose    = self._mp_pose.Pose(
            min_detection_confidence=CONF_THRESHOLD,
            min_tracking_confidence=CONF_THRESHOLD
        )
        self._mp_draw = mp.solutions.drawing_utils

        print("[CV] Webcam + MediaPipe initialized")

        # --- Main processing loop ---
        while self.running:
            try:
                ret, frame = self._cap.read()
            except Exception as e:
                print("[CV] ERROR reading frame:", e)
                time.sleep(0.05)
                continue


            if not ret:
                print("[CV] WARN: Failed to read frame.")
                time.sleep(0.05)
                continue

            # Convert BGR → RGB for MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process frame with mediapipe
            try:
                results = self._pose.process(rgb)
                angles = {} 
            except Exception as e:
                print("[CV] ERROR during MediaPipe processing:", e)
                continue

        
            # ---------------- ANGLE EXTRACTION ----------------
            angles = {}

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                # Extract raw coordinate triples (normalized)
                # Left side
                lh  = [lm[23].x, lm[23].y]
                ls  = [lm[11].x, lm[11].y]
                le  = [lm[13].x, lm[13].y]
                lw  = [lm[15].x, lm[15].y]
                lt  = [lm[21].x, lm[21].y]

                # Right side
                rh  = [lm[24].x, lm[24].y]
                rs  = [lm[12].x, lm[12].y]
                re  = [lm[14].x, lm[14].y]
                rw  = [lm[16].x, lm[16].y]
                rt  = [lm[22].x, lm[22].y]

                # Confidence threshold
                th = CONF_THRESHOLD

                # Left elbow
                if lm[11].visibility > th and lm[13].visibility > th and lm[15].visibility > th:
                    angles["left_elbow"] = calculate_angle(ls, le, lw)
                else:
                    angles["left_elbow"] = None

                # Left shoulder
                if lm[11].visibility > th and lm[13].visibility > th and lm[23].visibility > th:
                    angles["left_shoulder"] = calculate_angle(lh, ls, le)
                else:
                    angles["left_shoulder"] = None

                # Left wrist
                if lm[21].visibility > th and lm[13].visibility > th and lm[15].visibility > th:
                    angles["left_wrist"] = calculate_angle(le, lw, lt)
                else:
                    angles["left_wrist"] = None

                # Right elbow
                if lm[12].visibility > th and lm[14].visibility > th and lm[16].visibility > th:
                    angles["right_elbow"] = calculate_angle(rs, re, rw)
                else:
                    angles["right_elbow"] = None

                # Right shoulder
                if lm[12].visibility > th and lm[14].visibility > th and lm[24].visibility > th:
                    angles["right_shoulder"] = calculate_angle(rh, rs, re)
                else:
                    angles["right_shoulder"] = None

                # Right wrist
                if lm[22].visibility > th and lm[14].visibility > th and lm[16].visibility > th:
                    angles["right_wrist"] = calculate_angle(re, rw, rt)
                else:
                    angles["right_wrist"] = None

            else:
                angles = {
                    "left_elbow": None,
                    "left_shoulder": None,
                    "left_wrist": None,
                    "right_elbow": None,
                    "right_shoulder": None,
                    "right_wrist": None
                }

            self.latest_angles = self._smooth_angles(angles)


            # ---------------- DRAWING / OVERLAY ----------------
            image_bgr = frame.copy()   # draw on original BGR image (OpenCV style)

            if results.pose_landmarks:
                # Draw skeleton lines & points
                self._mp_draw.draw_landmarks(
                    image_bgr,
                    results.pose_landmarks,
                    self._mp_pose.POSE_CONNECTIONS
                )



            # ===== Draw angle text at joint pixel positions =====
            h, w, _ = image_bgr.shape

            def px(lm):
                return int(lm.x * w), int(lm.y * h)

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                # Left side pixel coords
                left_elbow_px     = px(lm[13])
                left_shoulder_px  = px(lm[11])
                left_wrist_px     = px(lm[15])

                # Right side pixel coords
                right_elbow_px    = px(lm[14])
                right_shoulder_px = px(lm[12])
                right_wrist_px    = px(lm[16])

                # RIGHT side (red)
                if angles.get("right_elbow") is not None:
                    cv2.putText(image_bgr, str(int(angles["right_elbow"])), right_elbow_px,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

                if angles.get("right_shoulder") is not None:
                    cv2.putText(image_bgr, str(int(angles["right_shoulder"])), right_shoulder_px,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

                # if angles.get("right_wrist") is not None:
                #     cv2.putText(image_bgr, str(int(angles["right_wrist"])), right_wrist_px,
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

                # LEFT side (blue)
                if angles.get("left_elbow") is not None:
                    cv2.putText(image_bgr, str(int(angles["left_elbow"])), left_elbow_px,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

                if angles.get("left_shoulder") is not None:
                    cv2.putText(image_bgr, str(int(angles["left_shoulder"])), left_shoulder_px,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

                # if angles.get("left_wrist") is not None:
                #     cv2.putText(image_bgr, str(int(angles["left_wrist"])), left_wrist_px,
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)


            # Convert BGR → RGB for GUI
            rgb_out = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

            # Store final output frame
            self.latest_frame = rgb_out


            time.sleep(TARGET_DT)

            # Prevent accidental infinite loop if running becomes False elsewhere
            if not self.running:
                break



        # --- Cleanup once stopped ---
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        if getattr(self, "_pose", None):
            try:
                self._pose.close()
            except Exception:
                pass
            self._pose = None
        self._mp_pose = None
        self._mp_draw = None
        self.latest_frame = None
        self.latest_angles = {}
        self.prev_angles = {}
        self.running = False
        self._thread = None
        cv2.destroyAllWindows()
        print("[CV] Stopped + cleaned up")






    def _smooth_angles(self, new_angles: dict, alpha: float = 0.25):
        """
        Smooth angles with simple exponential smoothing.
        alpha = 0.25 → 25 percent new, 75 percent previous.
        """
        out = {}

        for k, v in new_angles.items():
            if v is None:  # cannot smooth None values
                out[k] = None
                continue

            old = self.prev_angles.get(k, None)
            if old is None:
                out[k] = v
            else:
                out[k] = (1 - alpha) * old + alpha * v

        self.prev_angles = out
        return out

