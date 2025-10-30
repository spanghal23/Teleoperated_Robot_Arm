# cv_collision_combo.py — KINEMATIC ONLY: set 'base_yaw' directly from L_elbow (deg→rad)
import os, time, math, tempfile
from typing import Dict, Optional, List

import pybullet as p
import pybullet_data

# ===== CONFIG (no physics) =====
TIMESTEP     = 1.0 / 240.0
PRINT_HZ     = 10.0
CAM_INDEX    = 0
SHOW_CV      = True          # show the webcam window from starterCode.py
USE_PROCESS  = True          # run CV in a separate process (reliable on Windows)
INVERT_BASE  = False         # flip sign if desired

HERE = os.path.dirname(__file__)
URDF_CANDIDATES: List[str] = [
    os.path.join(HERE, "collision_detection", "urdf", "project_arm.urdf"),
    os.path.join(HERE, "urdf", "project_arm.urdf"),
    os.path.join(HERE, "project_arm.urdf"),
]

# Base revolute joint name in your URDF (about +Z)
BASE_JOINT_NAME = "base_yaw"
# ===============================

os.environ.setdefault("GLOG_minloglevel", "2")

# ---------- TOP-LEVEL CV WORKER (picklable on Windows) ----------
def cv_worker(angle_queue, cam_index: int, show: bool):
    """Runs in a separate process; forwards freshest angles via angle_queue."""
    from starterCode import read_webcam_fast
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
        from starterCode import read_webcam_fast
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
    # minimal GUI
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    return cid

def setup_world():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    # No gravity, no physics stepping needed beyond redraws
    p.setGravity(0, 0, 0)
    p.setRealTimeSimulation(0)  # we’ll call stepSimulation just to refresh the viewer
    p.setTimeStep(TIMESTEP)
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.7]
    )

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
    """Load URDF for visualization only (no reliance on inertials)."""
    try:
        with open(urdf_path, "r", encoding="utf-8") as f:
            txt = f.read()
    except Exception as e:
        print(f"[WARN] Failed to read URDF: {e}")
        return None

    # Keep it simple: no inertia flags. We’ll just reset joint states each frame.
    with tempfile.NamedTemporaryFile("w", suffix="_patched.urdf", delete=False) as tf:
        tf.write(txt)
        patched = tf.name

    flags = p.URDF_MAINTAIN_LINK_ORDER  # no URDF_USE_INERTIA_FROM_FILE, no motors needed
    try:
        uid = p.loadURDF(
            patched,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True,  # fixed base for a manipulator
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

# ---------- MAIN ----------
def main():
    latest_angles: Dict[str, float] = {}
    first_print = False

    def deliver(a: Dict[str, float]):
        nonlocal latest_angles, first_print
        if not a: return
        latest_angles = a
        if not first_print and "L_elbow" in a:
            print("[CV] First L_elbow:", a["L_elbow"])
            first_print = True

    # Start CV producer
    cv_handle = start_cv(USE_PROCESS, CAM_INDEX, SHOW_CV, deliver)

    # Render-only Bullet setup
    client = connect_gui()
    setup_world()
    add_box_obstacle()

    urdf_path = resolve_urdf_path()
    robot_uid = load_robot_kinematic(urdf_path) if urdf_path else None
    base_idx = None
    if robot_uid is not None:
        base_idx = get_joint_index_exact(robot_uid, BASE_JOINT_NAME)
        if base_idx is not None:
            info = p.getJointInfo(robot_uid, base_idx)
            name = info[1].decode("utf-8")
            axis = info[13]          # should be ~ (0,0,1)
            print(f"[CHECK] base_idx={base_idx}, name={name}, axis={axis} (kinematic mode)")

    print("[INFO] Running (kinematic). Close the Bullet window to exit.")
    last_log = 0.0

    while p.isConnected(client):
        # Show L_elbow for bring-up
        now = time.perf_counter()
        if "L_elbow" in latest_angles and (now - last_log) >= 1.0 / PRINT_HZ:
            print(f"L_elbow: {latest_angles['L_elbow']:.1f} deg")
            last_log = now

        # Kinematic update: set joint angle directly (no motors, no physics)
        if robot_uid is not None and base_idx is not None and "L_elbow" in latest_angles:
            deg = float(latest_angles["L_elbow"])
            rad = math.radians(deg)
            if INVERT_BASE: rad = -rad
            p.resetJointState(robot_uid, base_idx, rad)

        # Step just to refresh the viewer (no physics happening)
        p.stepSimulation()
        time.sleep(TIMESTEP)

    print("[INFO] Disconnected. Bye.")
    # No special cleanup needed; child process/thread will exit when Python exits.

if __name__ == "__main__":
    main()
