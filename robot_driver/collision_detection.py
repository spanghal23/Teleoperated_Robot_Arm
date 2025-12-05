# collision_detection.py
import numpy as np
import cv2
import math
import pybullet as p
import pybullet_data
import os


HERE = os.path.dirname(__file__)


# set to -1.0 if direction is backwards
ANGLE_SCALES = {
    "left_shoulder": 1.0,   
    "left_elbow":    1.0,
    "left_wrist":    1.0,

    "right_shoulder": 1.0,   
    "right_elbow":    1.0,
    "right_wrist":    1.0,
}

#  per-joint offsets and scale (for flipping direction / shifting neutral)
ANGLE_OFFSETS_DEG = {
    "left_shoulder": 0.0,   # e.g. -30.0 later if neutral is tilted
    "left_elbow":    0.0,
    "left_wrist":    0.0,

    "right_shoulder": 0.0,   
    "right_elbow":    0.0,
    "right_wrist":    0.0,
}

# Conservative hard limits for CV joints (degrees)
# SAFE_LIMITS_DEG clamps the robot’s behavior into your “allowed” joint range.
SAFE_LIMITS_DEG = {
    "left_shoulder": (-90.0, 90.0),   # node 1
    "left_elbow":    (-90.0, 90.0),   # node 2
    "left_wrist":    (-60.0, 60.0),

    "right_shoulder": (-60.0, 60.0),  # node 3 <- no collision detection but still processing 
    "right_elbow":    (-180,   180),    # node 0
    "right_wrist":   (-60.0, 60.0),
}


def remap(val, in_min, in_max, out_min, out_max):
    """Linearly map val from one range to another."""
    if val is None:
        return None
    # clamp to input range first
    if val < in_min: 
        val = in_min
    if val > in_max: 
        val = in_max
    # normalize
    t = (val - in_min) / (in_max - in_min)
    return out_min + t * (out_max - out_min)


class CollisionDetector:
    def __init__(self, urdf_path=None, use_gui=False, joint_map = None):
        # --- choose URDF path ---
        if urdf_path is None:
            urdf_path = os.path.join(HERE, "urdf", "project_arm.urdf")
        self.urdf_path = urdf_path

        self.use_gui = use_gui
        self.client = None
        self.robot = None

        # Mapping from CV joint names to robot URDF joint names
        # Easy to change later
        # Default mapping from CV joint names to URDF joint names
        self.joint_map = {
            "right_elbow":  "node0",   
            "left_shoulder": "node1",
            "left_elbow":    "node2",
        }

        # If caller supplies a custom mapping, use that instead
        if joint_map is not None:
            self.joint_map = joint_map


        # cache for last processed angles (for GUI debug)
        self.last_processed_angles = {}

        self._connect_and_load()


    def _connect_and_load(self):
        """Start PyBullet and load the robot + plane."""

        if self.use_gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)

       # Load plane
        self.plane = p.loadURDF("plane.urdf")

        # ---------- Make the plane a flat solid color instead of grid ----------
        p.changeVisualShape(
            self.plane,
            -1,                     # base link of plane URDF
            rgbaColor=[0.25, 0.25, 0.25, 1.0]   # clean medium-dark grey
        )

        # Load robot
        self.robot = p.loadURDF(
            self.urdf_path,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )

        # ---------- Recolor robot to a light neutral grey ----------
        LIGHT_GREY = [0.85, 0.85, 0.85, 1.0]   # change if you want slightly lighter/darker

        num_joints = p.getNumJoints(self.robot)

        # recolor base link (index -1)
        p.changeVisualShape(self.robot, -1, rgbaColor=LIGHT_GREY)

        # recolor all child links
        for j in range(num_joints):
            p.changeVisualShape(self.robot, j, rgbaColor=LIGHT_GREY)


        # Disable default motors so joint states are freely settable
        num_j = p.getNumJoints(self.robot)
        for j in range(num_j):
            p.setJointMotorControl2(
                self.robot, j,
                controlMode=p.VELOCITY_CONTROL,
                force=0
            )

        print("[CD] Loaded robot with 3 joints")


    def shutdown(self):
        if self.client is not None:
            p.disconnect(self.client)
            self.client = None


    def check(self, angles_deg):
        """
        angles_deg: dict of CV angles in degrees.
        Returns: (in_collision: bool, message: str)
        """

        # 1) process + apply robot pose
        proc = self._process_angles(angles_deg)
        self._apply_angles(proc)

        # 2) needed for internal collision update
        p.stepSimulation()

        # 3) check contacts
        contacts = p.getContactPoints(bodyA=self.robot)

        if len(contacts) == 0:
            return False, "OK"

        # 4) Build readable report
        lines = []
        for c in contacts:
            linkA = c[3]
            linkB = c[4]
            lines.append(f"🛑 COLLISION: link {linkA} with link {linkB}")

        return True, "\n".join(lines)


    # -----------------------
    #   preprocessing 
    # -----------------------
    def _process_angles(self, raw_angles_deg: dict) -> dict:
        """
        Take raw CV angles (deg) and return processed angles (deg) with:
          - remap for elbows (CV 35–165 -> robot 0–180)
          - scale/offset for others
          - hard clamp into SAFE_LIMITS_DEG
        Output keys are still CV joint names ("left_shoulder", etc.).
        """
        processed = {}

        for cv_name, urdf_joint in self.joint_map.items():
            val = raw_angles_deg.get(cv_name, None)
            if val is None:
                continue

            # remapping: map human CV angle range to robot joint range
            if cv_name in ["left_elbow"]:
                # 35–165 human range → -90–90 robot range
                ang = remap(val, 35.0, 165.0, -90.0, 90.0)

            elif cv_name in ["left_shoulder", "right_shoulder"]:
                # Shoulders are often similar but you can change this anytime
                ang = remap(val, 15.0, 180.0, -90.0, 90.0)

            elif cv_name in ["right_elbow"]:
                ang = remap(val, 35.0, 165.0, -90.0, 90.0)

            else:
                # Default case (wrist, etc.)
                ang = val

            # Apply sign flip and offset AFTER remapping
            ang = ang * ANGLE_SCALES[cv_name] + ANGLE_OFFSETS_DEG[cv_name]

            # clamp to safe limits
            amin, amax = SAFE_LIMITS_DEG.get(cv_name, (-180.0, 180.0))
            ang = max(amin, min(amax, ang))

            processed[cv_name] = ang

        # cache for GUI inspection
        self.last_processed_angles = processed
        return processed





    # -----------------------
    #   Joint Mapping Logic
    # -----------------------

    def _apply_angles(self, angles_deg: dict):
        """
        Apply CV angles (degrees) to the robot joints.
        Only joints defined in self.joint_map are used.
        Missing joints are ignored safely.
        """
        if self.robot is None:
            return

        for cv_name, urdf_joint in self.joint_map.items():
            if cv_name not in angles_deg:
                continue   # skip missing joints

            angle_deg = angles_deg[cv_name]
            if angle_deg is None:
                continue   # skip invalid ones

            # convert to radians
            angle_rad = angle_deg * (3.14159265 / 180.0)

            # get joint index
            j_idx = self._find_joint_index(urdf_joint)
            if j_idx is None:
                continue

            # apply to pybullet
            p.resetJointState(self.robot, j_idx, angle_rad)




    def _find_joint_index(self, joint_name: str):
        """
        Retrieve and cache joint indices for fast lookup.
        """
        if not hasattr(self, "_joint_index_cache"):
            # build cache
            cache = {}
            n = p.getNumJoints(self.robot)
            for j in range(n):
                nm = p.getJointInfo(self.robot, j)[1].decode("utf-8")
                cache[nm] = j
            self._joint_index_cache = cache

        return self._joint_index_cache.get(joint_name, None)
    



    def render(self, width=333, height=250):
        if self.robot is None:
            return None

        view_mat = p.computeViewMatrix(
            cameraEyePosition=[1.4, -1.4, 1.0],
            cameraTargetPosition=[0.0, 0.0, 0.4],
            cameraUpVector=[0, 0, 1]
        )

        proj_mat = p.computeProjectionMatrixFOV(
            fov=55,
            aspect=width/height,
            nearVal=0.01,
            farVal=5.0
        )

        w, h, rgba, depth, seg_mask = p.getCameraImage(
            width,
            height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # reshape & drop alpha
        rgb = np.array(rgba, dtype=np.uint8).reshape((h, w, 4))[:, :, :3]


        return rgb


    def get_state(self, angles_deg):
        """
        Update robot, run collision check, return (collision, report, image)
        """
        coll, rep = self.check(angles_deg)
        img = self.render()
        return coll, rep, img


