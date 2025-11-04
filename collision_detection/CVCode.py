# cv_pose_fast.py
import time
from typing import Callable, Dict, Optional

import cv2
import mediapipe as mp

# ---------- math helpers ----------
def _angle_3pt(a, b, c):
    """Angle ABC in degrees; a,b,c are (x,y) in normalized coords (0..1)."""
    import numpy as np, math
    a, b, c = map(lambda p: np.array(p, float), (a, b, c))
    ba, bc = a - b, c - b
    denom = (np.linalg.norm(ba) * np.linalg.norm(bc)) + 1e-9
    x = float((ba @ bc) / denom)
    x = max(-1.0, min(1.0, x))
    return math.degrees(math.acos(x))

def _open_camera(idx: int):
    """Try multiple Windows backends to reduce 'camera busy' issues."""
    tried = []
    for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, 0):  # 0 = auto
        cap = cv2.VideoCapture(idx, backend)
        ok = cap.isOpened()
        tried.append((backend, ok))
        if ok:
            return cap
        cap.release()
    print(f"[ERR] Could not open camera {idx}. Tried: {tried}")
    return None

# ---------- main fast reader ----------
def read_webcam_fast(
    cam_index: int = 0,
    on_angles: Optional[Callable[[Dict[str, float]], None]] = None,
    show: bool = True,
    process_scale: float = 0.5,   # downscale factor for MediaPipe (0.5 = 50%)
    drop_grabs: int = 1           # drop queued frames each loop to avoid latency
):
    """
    Low-latency pose angles reader.
    - Calls `on_angles(dict)` each frame (latest only).
    - If `show=True`, draws a tiny HUD (fast).
    """
    mp_pose = mp.solutions.pose

    cap = _open_camera(cam_index)
    if cap is None:
        print("[HINT] Close Zoom/Teams/Camera app; try cam_index 1/2; check Windows privacy settings.")
        return

    # Ask for a low-latency, low-cost stream (driver may ignore some)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # some backends ignore this

    if show:
        cv2.namedWindow("Pose Tracking", cv2.WINDOW_NORMAL)

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=0,          # <<< fastest model
        smooth_landmarks=True,
        enable_segmentation=False,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7
    )

    thr = 0.7  # visibility threshold

    try:
        while True:
            # Drop any backlog to get the freshest frame
            for _ in range(max(0, drop_grabs)):
                cap.grab()
            ok, frame = cap.retrieve()
            if not ok or frame is None:
                continue

            # Downscale for faster inference (angles are scale invariant)
            if process_scale != 1.0:
                small = cv2.resize(frame, None, fx=process_scale, fy=process_scale, interpolation=cv2.INTER_AREA)
            else:
                small = frame

            rgb_small = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            res = pose.process(rgb_small)

            angles: Dict[str, float] = {}

            if res and res.pose_landmarks:
                lms = res.pose_landmarks.landmark

                def pt(i): return (lms[i].x, lms[i].y)
                def vis(i): return getattr(lms[i], "visibility", 1.0) > thr

                # indices: https://developers.google.com/mediapipe/solutions/vision/pose
                Ls, Le, Lw = pt(11), pt(13), pt(15)
                Rs, Re, Rw = pt(12), pt(14), pt(16)
                Lh, Rh     = pt(23), pt(24)
                Lt, Rt     = pt(21), pt(22)  # if you want wrist angles

                angles["L_elbow"]    = _angle_3pt(Ls, Le, Lw) if (vis(11) and vis(13) and vis(15)) else 0.0
                angles["L_shoulder"] = _angle_3pt(Lh, Ls, Le) if (vis(23) and vis(11) and vis(13)) else 0.0
                angles["R_elbow"]    = _angle_3pt(Rs, Re, Rw) if (vis(12) and vis(14) and vis(16)) else 0.0
                angles["R_shoulder"] = _angle_3pt(Rh, Rs, Re) if (vis(24) and vis(12) and vis(14)) else 0.0
                # Example if needed:
                # angles["L_wrist"] = _angle_3pt(Le, Lw, Lt) if (vis(13) and vis(15) and vis(21)) else 0.0

            if on_angles:
                # Hand off immediately (let the consumer decide control rate)
                on_angles(angles)

            if show:
                # Very light HUD (fast; skip skeleton drawing to save time)
                text = f"L(elb:{angles.get('L_elbow',0):.0f}, shd:{angles.get('L_shoulder',0):.0f})  " \
                       f"R(elb:{angles.get('R_elbow',0):.0f}, shd:{angles.get('R_shoulder',0):.0f})"
                img = frame
                cv2.putText(img, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.imshow("Pose Tracking", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                # Still pump GUI events a little (some backends need it)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()
        pose.close()

# Quick manual test:
if __name__ == "__main__":
    def _print_angles(a): 
        # super-thin print; comment out for max speed
        print(a)
    read_webcam_fast(cam_index=0, on_angles=_print_angles, show=True, process_scale=0.5, drop_grabs=1)
