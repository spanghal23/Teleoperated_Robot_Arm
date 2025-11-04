# cv_collision_combo.py — KINEMATIC ONLY: set 'base_yaw' from L_elbow, plus map R_shoulder/R_elbow
import os, time, math, tempfile, sys, re
from typing import Dict, Optional, List, Tuple

import pybullet as p
import pybullet_data

# ===== CONFIG (no physics) =====
TIMESTEP     = 1.0 / 240.0
PRINT_HZ     = 10.0
CAM_INDEX    = 0
SHOW_CV      = True          # show the webcam window from CVCode.py
USE_PROCESS  = True          # run CV in a separate process (reliable on Windows)

# Inversion flags per joint (tune if your angle direction is flipped)
INVERT_BASE      = False
INVERT_SHOULDER  = False
INVERT_ELBOW     = False

# If your CV pipeline uses different names, edit here:
JOINT_KEYS = {
    "L_elbow":    "L_elbow",     # drives base_yaw (existing behavior)
    "R_shoulder": "R_shoulder",  # drives shoulder joint
    "R_elbow":    "R_elbow",     # drives elbow joint
}

# Optional: confidence suffixes if you pass them through your callback (safe to leave)
VIS_SUFFIX = "_vis"
PRS_SUFFIX = "_prs"

HERE = os.path.dirname(__file__)
URDF_CANDIDATES: List[str] = [
    os.path.join(HERE, "collision_detection", "urdf", "project_arm.urdf"),
    os.path.join(HERE, "urdf", "project_arm.urdf"),
    os.path.join(HERE, "project_arm.urdf"),
]

# ---- Name your URDF joints here ----
BASE_JOINT_NAME     = "base_yaw"      # about +Z
SHOULDER_JOINT_NAME = "side_hinge"    # about +X
ELBOW_JOINT_NAME    = "elbow_hinge"   # about +X

# ---- Self-collision scope (ONLY end link vs base) ----
END_LINK_NAME       = "link2"         # child link name of your elbow end (edit if different)

# Optional: flip if directions feel backwards
INVERT_BASE      = False
INVERT_SHOULDER  = False
INVERT_ELBOW     = False

# ====================================

os.environ.setdefault("GLOG_minloglevel", "2")

# ---------- TOP-LEVEL CV WORKER (picklable on Windows) ----------
def cv_worker(angle_queue, cam_index: int, show: bool):
    from CVCode import read_webcam_fast
    def on_angles_cb(a: Dict[str, float]):
        if not a: return
        try:
            while True: angle_queue.get_nowait()
        except Exception:
            pass
        try:
            angle_queue.put_nowait(a)
        except Exception:
            pass
    read_webcam_fast(cam_index=cam_index, on_angles=on_angles_cb, show=show)

# ---------- CV wiring (process OR thread) ----------
def start_cv(use_process: bool, cam_index: int, show: bool, deliver):
    if use_process:
        from multiprocessing import Process, Queue, set_start_method
        try:
            set_start_method("spawn", force=True)
        except RuntimeError:
            pass
        q: "Queue[Dict[str, float]]" = Queue(maxsize=1)
        proc = Process(target=cv_worker, args=(q, cam_index, show), daemon=True)
        proc.start()

        import threading
        def pump():
            while True:
                try:
                    a = q.get(timeout=0.2)
                    deliver(a)
                except Exception:
                    pass
        threading.Thread(target=pump, daemon=True).start()
        return proc
    else:
        import threading
        from CVCode import read_webcam_fast
        th = threading.Thread(
            target=lambda: read_webcam_fast(cam_index=cam_index, on_angles=deliver, show=show),
            daemon=True
        )
        th.start()
        return th

# ---------- PyBullet (render-only, kinematic) ----------
def connect_gui() -> int:
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError("Failed to connect to PyBullet GUI.")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    return cid

def setup_world() -> int:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, 0)
    p.setRealTimeSimulation(0)
    p.setTimeStep(TIMESTEP)
    plane_uid = p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.7]
    )
    return plane_uid

def add_box_obstacle(
    half=(0.12, 0.12, 0.20), rgba=(0.2, 0.7, 0.9, 1.0), pos=(0.6, 0.0, 0.20), orn=(0, 0, 0, 1)
):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    uid = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                            basePosition=pos, baseOrientation=orn)
    print("[INFO] Obstacle uid:", uid)
    return uid

def resolve_urdf_path() -> Optional[str]:
    for path in URDF_CANDIDATES:
        if os.path.exists(path):
            print("[INFO] Using URDF:", path)
            return path
    print("[WARN] project_arm.urdf not found. Tried:")
    for path in URDF_CANDIDATES:
        print("  -", path)
    return None

def load_robot_kinematic(urdf_path: str) -> Optional[int]:
    """Load URDF for visualization, with self-collision enabled, writing UTF-8 safely on Windows."""
    # Read as UTF-8 (tolerate weird files by replacing bad bytes)
    try:
        with open(urdf_path, "r", encoding="utf-8", errors="replace") as f:
            txt = f.read()
    except Exception as e:
        print(f"[WARN] Failed to read URDF: {e}")
        return None

    # Write a patched copy as UTF-8 so PyBullet can read it (Windows default cp1252 would fail)
    try:
        with tempfile.NamedTemporaryFile("w", suffix="_patched.urdf", delete=False, encoding="utf-8", newline="\n") as tf:
            tf.write(txt)
            patched = tf.name
    except Exception as e:
        print(f"[WARN] Failed to write patched URDF as UTF-8: {e}")
        # Fallback: write bytes (always safe)
        try:
            with tempfile.NamedTemporaryFile("wb", suffix="_patched.urdf", delete=False) as tfb:
                tfb.write(txt.encode("utf-8", errors="replace"))
                patched = tfb.name
        except Exception as e2:
            print(f"[ERROR] Failed to write patched URDF in binary: {e2}")
            return None

    # Enable self-collision (or your existing flags)
    flags = p.URDF_MAINTAIN_LINK_ORDER | p.URDF_USE_SELF_COLLISION
    try:
        uid = p.loadURDF(
            patched,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True,
            flags=flags,
        )
        print(f"[INFO] Robot uid={uid}, joints={p.getNumJoints(uid)}")
        return uid
    except Exception as e:
        print(f"[WARN] Failed to load URDF: {e}")
        return None

def get_joint_index_exact(body_uid: int, joint_name: str) -> Optional[int]:
    names = {}
    n = p.getNumJoints(body_uid)
    for j in range(n):
        nm = p.getJointInfo(body_uid, j)[1].decode("utf-8")
        names[nm] = j
    if joint_name in names:
        print(f"[INFO] Using joint '{joint_name}' -> index {names[joint_name]}")
        return names[joint_name]
    for nm, j in names.items():
        if nm.lower() == joint_name.lower():
            print(f"[INFO] Using joint '{nm}' (ci match) -> index {j}")
            return j
    print("[ERROR] Joint not found:", joint_name)
    print("        Available joints:")
    for nm, j in names.items():
        print(f"         - {j}: {nm}")
    return None

def wrap_deg_0_360(deg: float) -> float:
    w = deg % 360.0
    return w if w < 360.0 else 0.0

# ---------- Collision UI / helpers ----------
def create_status_marker() -> Tuple[int, int]:
    side_pos = (-0.7, -0.6, 0.1)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.02, 0.02, 0.10))
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.02, 0.02, 0.10), rgbaColor=(0.1, 0.8, 0.1, 1.0))
    pylon_uid = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                  basePosition=side_pos, baseOrientation=(0, 0, 0, 1))
    text_id = p.addUserDebugText("NOMINAL", [side_pos[0], side_pos[1], side_pos[2]+0.25],
                                 textSize=1.5, textColorRGB=[0.1, 0.8, 0.1])
    return pylon_uid, text_id

def update_status_marker(pylon_uid: int, text_id: int, in_collision: bool):
    color = (0.85, 0.1, 0.1, 1.0) if in_collision else (0.1, 0.8, 0.1, 1.0)
    label = "IN COLLISION" if in_collision else "NOMINAL"
    p.changeVisualShape(pylon_uid, -1, rgbaColor=color)
    p.removeUserDebugItem(text_id)
    pos, _ = p.getBasePositionAndOrientation(pylon_uid)
    new_id = p.addUserDebugText(label, [pos[0], pos[1], pos[2]+0.25],
                                textSize=1.5, textColorRGB=color[:3])
    return new_id

def build_link_name_map(body_uid: int) -> Dict[int, str]:
    m = {-1: "base_link"}
    n = p.getNumJoints(body_uid)
    for j in range(n):
        info = p.getJointInfo(body_uid, j)
        link_name = info[12].decode("utf-8")  # child link name
        m[j] = link_name
    return m

def get_link_index_by_name(body_uid: int, child_link_name: str) -> Optional[int]:
    n = p.getNumJoints(body_uid)
    for j in range(n):
        info = p.getJointInfo(body_uid, j)
        if info[12].decode("utf-8") == child_link_name:
            return j
    return None

def check_collisions(robot_uid: int, plane_uid: int, obstacle_uids: List[int], end_link_idx: Optional[int]) -> Tuple[bool, str]:
    """Return (in_collision, description). Self-collision ONLY: end_link vs base_link."""
    desc_lines = []

    # robot vs ground
    for c in p.getContactPoints(bodyA=robot_uid, bodyB=plane_uid):
        a_l, b_l = c[3], c[4]
        desc_lines.append(f"Robot[{a_l}] vs Ground[{b_l}] (normalForce={c[9]:.3f})")

    # robot vs obstacles
    for oid in obstacle_uids:
        for c in p.getContactPoints(bodyA=robot_uid, bodyB=oid):
            a_l, b_l = c[3], c[4]
            desc_lines.append(f"Robot[{a_l}] vs Obstacle({oid})[{b_l}] (normalForce={c[9]:.3f})")

    # self-collision: ONLY end link vs base (-1)
    if end_link_idx is not None:
        for c in p.getContactPoints(bodyA=robot_uid, bodyB=robot_uid):
            a_l, b_l = c[3], c[4]
            if {a_l, b_l} == {end_link_idx, -1}:
                desc_lines.append(f"Robot[{a_l}] vs Robot[{b_l}] (normalForce={c[9]:.3f})")

    if not desc_lines:
        return False, ""

    # pretty-print with link names
    names = build_link_name_map(robot_uid)
    pretty = []
    for line in desc_lines:
        def repl(m):
            idx = int(m.group(1))
            return f"Robot[{names.get(idx, idx)}]"
        pretty.append(re.sub(r"Robot\[(\-?\d+)\]", repl, line))
    return True, "Contacts:\n" + "\n".join(pretty)

def show_collision_dialog_and_exit(message: str):
    try:
        import tkinter as tk
        from tkinter import messagebox
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("Collision detected", message + "\n\nClick OK to close.")
    except Exception as e:
        print("[COLLISION]", message)
        print("[WARN] Could not show Tk dialog:", e)
    try:
        if p.isConnected():
            p.disconnect()
    finally:
        sys.exit(0)

# ---------- Helpers for printing ----------
def _fmt_joint_line(all_angles: Dict[str, float], all_conf: Dict[str, Dict[str, float]]) -> str:
    parts = []
    for key in (JOINT_KEYS["L_elbow"], JOINT_KEYS["R_shoulder"], JOINT_KEYS["R_elbow"]):
        if key in all_angles:
            deg = float(all_angles[key])
            c = all_conf.get(key, {})
            vis = c.get("vis", None)
            prs = c.get("prs", None)
            if vis is not None or prs is not None:
                s = f"{key}: {deg:.1f}°"
                if vis is not None and prs is not None:
                    s += f" (vis={vis:.2f} prs={prs:.2f})"
                elif vis is not None:
                    s += f" (vis={vis:.2f})"
                elif prs is not None:
                    s += f" (prs={prs:.2f})"
                parts.append(s)
            else:
                parts.append(f"{key}: {deg:.1f}°")
    return " | ".join(parts) if parts else ""

# ---------- MAIN ----------
def main():
    latest_angles: Dict[str, float] = {}
    latest_conf: Dict[str, Dict[str, float]] = {}
    first_print = False

    def deliver(a: Dict[str, float]):
        nonlocal latest_angles, latest_conf, first_print
        if not a: return
        for logical, key in JOINT_KEYS.items():
            if key in a:
                latest_angles[key] = float(a[key])
            vis_key = key + VIS_SUFFIX
            prs_key = key + PRS_SUFFIX
            if vis_key in a or prs_key in a:
                latest_conf[key] = {
                    "vis": float(a.get(vis_key, latest_conf.get(key, {}).get("vis", float("nan")))),
                    "prs": float(a.get(prs_key, latest_conf.get(key, {}).get("prs", float("nan")))),
                }
        if not first_print and JOINT_KEYS["L_elbow"] in latest_angles:
            print("[CV] First L_elbow:", latest_angles[JOINT_KEYS["L_elbow"]])
            first_print = True

    # Start CV producer
    cv_handle = start_cv(USE_PROCESS, CAM_INDEX, SHOW_CV, deliver)

    # Render-only Bullet setup
    client = connect_gui()
    plane_uid = setup_world()
    obstacle_uids: List[int] = []
    obstacle_uids.append(add_box_obstacle())

    urdf_path = resolve_urdf_path()
    robot_uid = load_robot_kinematic(urdf_path) if urdf_path else None

    # --- Resolve joint indices ---
    base_idx = shoulder_idx = elbow_idx = None
    end_link_idx = None
    if robot_uid is not None:
        base_idx     = get_joint_index_exact(robot_uid, BASE_JOINT_NAME)
        shoulder_idx = get_joint_index_exact(robot_uid, SHOULDER_JOINT_NAME)
        elbow_idx    = get_joint_index_exact(robot_uid, ELBOW_JOINT_NAME)
        end_link_idx = get_link_index_by_name(robot_uid, END_LINK_NAME)

        if base_idx is not None:
            axis = p.getJointInfo(robot_uid, base_idx)[13]
            print(f"[CHECK] base_idx={base_idx}, axis={axis} (kinematic mode)")
        if shoulder_idx is not None:
            axis = p.getJointInfo(robot_uid, shoulder_idx)[13]
            print(f"[CHECK] shoulder_idx={shoulder_idx}, axis={axis}")
        if elbow_idx is not None:
            axis = p.getJointInfo(robot_uid, elbow_idx)[13]
            print(f"[CHECK] elbow_idx={elbow_idx}, axis={axis}")
        print(f"[CHECK] end_link_idx={end_link_idx} (name='{END_LINK_NAME}')")

    # Create side status marker
    status_pylon_uid, status_text_id = create_status_marker()

    print("[INFO] Running (kinematic). Close the Bullet window to exit.")
    last_log = 0.0

    while p.isConnected(client):
        # Status print at PRINT_HZ
        now = time.perf_counter()
        if (now - last_log) >= 1.0 / PRINT_HZ:
            line = _fmt_joint_line(latest_angles, latest_conf)
            if line:
                print(line)
            last_log = now

        # ======= Kinematic updates =======
        key = JOINT_KEYS["L_elbow"]
        if robot_uid is not None and base_idx is not None and key in latest_angles:
            deg = 2.0*float(latest_angles[key])
            rad = math.radians(-deg if INVERT_BASE else deg)
            p.resetJointState(robot_uid, base_idx, rad)

        rsh_key = JOINT_KEYS["R_shoulder"]
        if robot_uid is not None and shoulder_idx is not None and rsh_key in latest_angles:
            deg = float(latest_angles[rsh_key]) - 90.0
            rad = math.radians(-deg if INVERT_SHOULDER else deg)
            p.resetJointState(robot_uid, shoulder_idx, rad)

        rel_key = JOINT_KEYS["R_elbow"]
        if robot_uid is not None and elbow_idx is not None and rel_key in latest_angles:
            deg = float(latest_angles[rel_key]) - 90.0
            rad = math.radians(-deg if INVERT_ELBOW else deg)
            p.resetJointState(robot_uid, elbow_idx, rad)

        # ---- Collision monitoring + UI ----
        in_collision, report = check_collisions(robot_uid, plane_uid, obstacle_uids, end_link_idx)
        status_text_id = update_status_marker(status_pylon_uid, status_text_id, in_collision)

        if in_collision:
            show_collision_dialog_and_exit(f"The robot is in collision.\n\n{report}")
            break

        # Step just to refresh the viewer (no physics happening)
        p.stepSimulation()
        time.sleep(TIMESTEP)

    print("[INFO] Disconnected. Bye.")

if __name__ == "__main__":
    main()
