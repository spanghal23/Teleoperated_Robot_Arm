# robot_collision.py
import os
import time
import threading
from typing import Dict, List

import pybullet as p
import pybullet_data

# ======== CONFIG ========
SHOW_CV = False          # set True to open the camera GUI window
CAM_INDEX = 0            # try 1/2 if you have multiple cameras
GRAVITY = -9.81
TIMESTEP = 1.0 / 240.0   # PyBullet default
# If you want to load your robot here, set a path; else load later in your own code.
ROBOT_URDF = ""          # e.g., r"C:\path\to\ur5.urdf" or keep "" to skip loading
# ========================

# Import your CV producer
from starterCode import read_webcam_fast  # must exist in the same folder


# ------------- CV THREAD: latest angles -------------
angles_latest: Dict[str, float] = {}
angles_lock = threading.Lock()

def on_angles_cb(a: Dict[str, float]):
    """Receive newest angles from the CV thread and store safely."""
    # Mutate in place (avoids swapping refs mid-read)
    with angles_lock:
        angles_latest.clear()
        if a:
            angles_latest.update(a)

def start_cv_thread():
    """Start the webcam/MediaPipe loop in a background thread."""
    t = threading.Thread(
        target=lambda: read_webcam_fast(cam_index=CAM_INDEX, on_angles=on_angles_cb, show=SHOW_CV),
        daemon=True
    )
    t.start()
    return t


# ------------- PyBullet helpers -------------
def connect_gui() -> int:
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError("Failed to connect to PyBullet GUI (p.GUI).")
    return cid

def setup_world():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, GRAVITY)
    p.setTimeStep(TIMESTEP)
    plane_id = p.loadURDF("plane.urdf")
    return plane_id

def load_robot(urdf_path: str):
    """Load your robot if a URDF path is provided. Return uid or None."""
    if not urdf_path:
        return None
    if not os.path.exists(urdf_path):
        print(f"[WARN] URDF not found: {urdf_path}. Continuing without robot.")
        return None
    base_pos = [0, 0, 0]
    base_orn = [0, 0, 0, 1]
    flags = p.URDF_USE_SELF_COLLISION
    try:
        uid = p.loadURDF(urdf_path, base_pos, base_orn, useFixedBase=True, flags=flags)
        return uid
    except Exception as e:
        print(f"[WARN] Failed to load URDF: {e}. Continuing without robot.")
        return None

def create_joint_sliders(body_uid: int) -> List[int]:
    """Create sliders for all revolute/prismatic joints and return their IDs."""
    slider_ids: List[int] = []
    if body_uid is None:
        return slider_ids

    n_joints = p.getNumJoints(body_uid)
    for j in range(n_joints):
        ji = p.getJointInfo(body_uid, j)
        # ji: (0 idx, 1 name, 2 type, ..., 8 ll, 9 ul)
        name = ji[1].decode("utf-8")
        jtype = ji[2]
        ll, ul = ji[8], ji[9]
        if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            # sane defaults if limits are huge/unused
            rmin = ll if ll > -1e6 else -3.14
            rmax = ul if ul < 1e6 else  3.14
            sid = p.addUserDebugParameter(f"Joint {j}: {name}", rmin, rmax, 0.0)
            slider_ids.append(sid)
    return slider_ids

def read_sliders_safe(slider_ids: List[int]) -> List[float]:
    """Read slider values; if invalid, raise to trigger a recreate."""
    vals: List[float] = []
    for sid in slider_ids:
        if not isinstance(sid, int) or sid < 0:
            raise p.error("Invalid slider id")
        vals.append(p.readUserDebugParameter(sid))
    return vals


# ------------- MAIN -------------
def main():
    # 1) Start CV thread first (so angles can flow immediately)
    cv_thread = start_cv_thread()

    # 2) Connect PyBullet GUI & build world
    client = connect_gui()
    setup_world()
    robot_uid = load_robot(ROBOT_URDF)

    # 3) Create sliders once
    slider_ids = create_joint_sliders(robot_uid)

    print("[INFO] Running. Press 'q' in the CV window (if shown) or close the Bullet GUI to exit.")
    last_print = 0.0

    while p.isConnected(client):
        # If you reset the sim elsewhere, slider IDs become invalid — handle gracefully
        try:
            joint_positions = read_sliders_safe(slider_ids)
        except p.error:
            # Rebuild sliders (e.g., after a resetSimulation)
            slider_ids = create_joint_sliders(robot_uid)
            time.sleep(0.01)
            continue

        # ---- DEMO: print L_elbow if present (throttled) ----
        with angles_lock:
            L_elb = angles_latest.get("L_elbow", None)
        now = time.perf_counter()
        if L_elb is not None and (now - last_print) > 0.1:  # 10 Hz print
            print(f"L_elbow: {L_elb:.1f} deg")
            last_print = now

        # ---- (Optional) Use joint_positions to control your robot here ----
        # Example:
        # if robot_uid is not None and joint_positions:
        #     p.setJointMotorControlArray(
        #         bodyUniqueId=robot_uid,
        #         jointIndices=[j for j in range(len(joint_positions))],
        #         controlMode=p.POSITION_CONTROL,
        #         targetPositions=joint_positions
        #     )

        p.stepSimulation()
        time.sleep(TIMESTEP)

    print("[INFO] Disconnected. Shutting down.")

if __name__ == "__main__":
    main()
