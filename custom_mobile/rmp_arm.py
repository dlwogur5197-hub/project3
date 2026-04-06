import os
import re
import time
import tempfile
import numpy as np

from .compat import get_motion_generation
from .params import RmpArmFollowParams
from .usd_utils import xform_world_pos, xform_world_pose, xform_world_quat_xyzw, ensure_target_cube


def _safe_read_text(path_str: str) -> str:
    with open(path_str, "r", encoding="utf-8") as f:
        return f.read()


def _safe_write_text(path_str: str, txt: str):
    with open(path_str, "w", encoding="utf-8") as f:
        f.write(txt)


def _regex_replace_joint_name_tokens(text: str, src_to_dst: dict) -> str:
    out = text
    for src, dst in src_to_dst.items():
        out = re.sub(rf"(?<![A-Za-z0-9_]){re.escape(src)}(?![A-Za-z0-9_])", dst, out)
    return out


class RmpArmFollower:
    """
    ARM-only RMP controller for custom_mobile (dummy base fixed, UR arm moves).

    Responsibilities:
      - stock UR5/UR5e RMP config load
      - joint-name patch to ur_arm_* naming
      - RMP base pose sync from UR arm base link prim
      - target world -> RMP target transform (scale + z bias)
      - articulation action generation + apply
      - optional base lock before/after action
    """

    def __init__(self, params: RmpArmFollowParams | None = None):
        self.params = params or RmpArmFollowParams()

        self.rig = None
        self.world = None
        self.stage = None

        self.mg = get_motion_generation()
        self.rmp = None
        self.art_rmp = None
        self.rmp_cfg = None
        self.rmp_robot_name = None
        self.rmp_ready = False

        self._tmp_dir = None
        self._tmp_urdf = None
        self._tmp_desc = None

        self._last_dbg_t = 0.0

    def bind(self, rig, world=None, stage=None):
        self.rig = rig
        self.world = world if world is not None else getattr(rig, "world", None)
        self.stage = stage if stage is not None else getattr(rig, "stage", None)
        if self.stage is None:
            raise RuntimeError("[RmpArmFollower] stage is None. Bind with stage or rig.stage.")
        return self

    def ensure_target(self):
        if self.params.create_target_if_missing and self.world is not None:
            ensure_target_cube(
                self.world,
                prim_path=self.params.target_prim_path,
                position=self.params.target_init_pos,
                scale=self.params.target_init_scale,
            )

    def initialize(self, physics_dt: float | None = None):
        if self.rig is None or self.rig.robot is None:
            raise RuntimeError("[RmpArmFollower] bind(rig) and rig.initialize() first.")

        self.ensure_target()

        if self.rig.base_q_lock is None:
            self.rig.snapshot_holds()

        cfg = self._load_stock_ur5_rmp_cfg()
        cfg = self._patch_ur5_joint_names_for_custom_articulation(cfg)
        self.rmp_cfg = cfg

        dt = self.params.physics_dt_default if physics_dt is None else float(physics_dt)

        self.rmp = self.mg.lula.motion_policies.RmpFlow(**cfg)
        self.art_rmp = self.mg.ArticulationMotionPolicy(self.rig.robot, self.rmp, dt)

        p_base, q_base = self.sync_rmp_base_pose()
        if p_base is not None:
            print(f"[RmpArmFollower] base prim = {self.params.ur_arm_base_link_prim_path}")
            print(f"[RmpArmFollower] base_pos = {np.round(p_base, 4).tolist()}")

        self.rmp_ready = True
        print(
            "[RmpArmFollower] initialized "
            f"(scale_comp={self.params.scale_comp:.4f}, z_bias={self.params.z_bias:.4f})"
        )
        return self

    # ---------- config patch ----------
    def _load_stock_ur5_rmp_cfg(self):
        tries = ["UR5", "UR5e"]
        last_err = None
        for robot_name in tries:
            try:
                cfg = self.mg.interface_config_loader.load_supported_motion_policy_config(robot_name, "RMPflow")
                self.rmp_robot_name = robot_name
                print(f"[RmpArmFollower] loaded stock RMP config: {robot_name}")
                return cfg
            except Exception as e:
                last_err = e
        raise RuntimeError(f"[RmpArmFollower] failed to load stock UR5/UR5e RMP config: {last_err}")

    def _patch_ur5_joint_names_for_custom_articulation(self, cfg: dict) -> dict:
        stock_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        mapping = {j: f"ur_arm_{j}" for j in stock_joints}

        urdf_path = cfg.get("urdf_path")
        desc_path = cfg.get("robot_description_path")
        if not urdf_path or not desc_path:
            raise RuntimeError(f"[RmpArmFollower] unexpected RMP config keys: {list(cfg.keys())}")

        urdf_txt = _safe_read_text(urdf_path)
        desc_txt = _safe_read_text(desc_path)

        urdf_txt2 = _regex_replace_joint_name_tokens(urdf_txt, mapping)
        desc_txt2 = _regex_replace_joint_name_tokens(desc_txt, mapping)

        self._tmp_dir = tempfile.mkdtemp(prefix="custom_mobile_ur5_rmp_")
        self._tmp_urdf = os.path.join(self._tmp_dir, os.path.basename(urdf_path))
        self._tmp_desc = os.path.join(self._tmp_dir, os.path.basename(desc_path))
        _safe_write_text(self._tmp_urdf, urdf_txt2)
        _safe_write_text(self._tmp_desc, desc_txt2)

        patched = dict(cfg)
        patched["urdf_path"] = self._tmp_urdf
        patched["robot_description_path"] = self._tmp_desc
        if "end_effector_frame_name" not in patched:
            patched["end_effector_frame_name"] = "wrist_3_link"
        return patched

    # ---------- pose sync / target transform ----------
    def _get_rmp_base_pose_from_stage(self):
        p, _ = xform_world_pose(self.stage, self.params.ur_arm_base_link_prim_path)
        if p is None:
            return None, None
        q = xform_world_quat_xyzw(self.stage, self.params.ur_arm_base_link_prim_path)
        return p, q

    def sync_rmp_base_pose(self):
        if self.rmp is None:
            return None, None

        p, q = self._get_rmp_base_pose_from_stage()
        if p is None:
            try:
                p, q = self.rig.robot.get_world_pose()
                p = np.array(p, dtype=np.float64)
                q = np.array(q, dtype=np.float64)
            except Exception:
                return None, None

        try:
            self.rmp.set_robot_base_pose(robot_position=p, robot_orientation=q)
        except TypeError:
            self.rmp.set_robot_base_pose(p, q)
        return np.array(p, dtype=np.float64), np.array(q, dtype=np.float64)

    def compute_rmp_target(self, p_tgt_world: np.ndarray, p_base_world: np.ndarray) -> np.ndarray:
        p_tgt_world = np.array(p_tgt_world, dtype=np.float64)
        p_base_world = np.array(p_base_world, dtype=np.float64)
        p_rmp = p_base_world + (p_tgt_world - p_base_world) * float(self.params.scale_comp)
        p_rmp[2] += float(self.params.z_bias)
        return p_rmp

    # ---------- control step ----------
    def step(self, dt: float | None = None):
        if not self.rmp_ready or self.rig is None or self.stage is None:
            return

        dt_val = self.params.physics_dt_default if (dt is None or dt <= 0) else float(dt)

        p_tgt_world = xform_world_pos(self.stage, self.params.target_prim_path)
        if p_tgt_world is None:
            return

        self.rig.apply_base_lock()  # before action

        p_base_world, _ = self.sync_rmp_base_pose()
        if p_base_world is None:
            return

        p_tgt_rmp = self.compute_rmp_target(p_tgt_world, p_base_world)

        self.rmp.set_end_effector_target(np.array(p_tgt_rmp, dtype=np.float64), None)
        self.rmp.update_world()

        try:
            action = self.art_rmp.get_next_articulation_action(dt_val)
        except TypeError:
            action = self.art_rmp.get_next_articulation_action()

        self.rig.robot.apply_action(action)

        if self.params.base_lock_after_action:
            self.rig.apply_base_lock()

        now = time.time()
        if now - self._last_dbg_t > self.params.debug_period:
            self._last_dbg_t = now
            ee = xform_world_pos(self.stage, self.params.ee_debug_prim_path)
            q = self.rig.get_q()
            q_base = [float(q[i]) for i in self.rig.base_idx] if self.rig.base_idx is not None else None
            q_arm = [float(q[i]) for i in self.rig.arm_idx] if self.rig.arm_idx else None
            print("[RmpArmFollower] follow")
            print(f"  target_world = {np.round(p_tgt_world, 4).tolist()}")
            print(f"  target_rmp   = {np.round(p_tgt_rmp, 4).tolist()}")
            print(f"  ee_debug     = {None if ee is None else np.round(ee, 4).tolist()}")
            print(f"  q_base       = {None if q_base is None else [round(v,4) for v in q_base]}")
            if q_arm is not None:
                print(f"  q_arm        = {[round(v,4) for v in q_arm]}")

    def cleanup(self):
        self.rmp_ready = False
        self.rmp = None
        self.art_rmp = None
        self.rmp_cfg = None
