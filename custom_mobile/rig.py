import numpy as np
import omni.usd

from .compat import get_core_classes
from .params import MoveBaseSquareParams, RmpArmFollowParams


class CustomMobileRig:
    """
    Reusable wrapper for custom_mobile articulation (/ridgeback).
    Handles:
      - loading USD into stage
      - articulation wrapper creation
      - DOF map discovery
      - base DOF lock
      - non-base DOF hold snapshot
    """

    def __init__(self, robot_prim_path="/ridgeback", robot_name="custom_mobile_ur5",
                 base_joint_names=None):
        self.robot_prim_path = robot_prim_path
        self.robot_name = robot_name
        self.base_joint_names = list(base_joint_names or [
            "dummy_base_prismatic_x_joint",
            "dummy_base_prismatic_y_joint",
            "dummy_base_revolute_z_joint",
        ])

        self.world = None
        self.stage = None
        self.robot = None

        self.dof_names = None
        self.base_idx = None
        self.arm_idx = None
        self.other_idx = None

        self.base_q_lock = None
        self.q_hold_other = None

    @classmethod
    def from_params(cls, params):
        if isinstance(params, (MoveBaseSquareParams, RmpArmFollowParams)):
            return cls(
                robot_prim_path=params.robot_prim_path,
                robot_name=params.robot_name,
                base_joint_names=params.base_joint_names,
            )
        return cls()

    def add_to_scene(self, world, usd_path: str):
        """
        Add custom_mobile USD to stage and wrap articulation.
        """
        SingleArticulation, add_reference_to_stage, _ = get_core_classes()

        self.world = world
        self.stage = omni.usd.get_context().get_stage()

        add_reference_to_stage(usd_path=usd_path, prim_path=self.robot_prim_path)
        world.scene.add(SingleArticulation(prim_path=self.robot_prim_path, name=self.robot_name))
        return self

    def attach_existing(self, world):
        self.world = world
        self.stage = omni.usd.get_context().get_stage()
        self.robot = world.scene.get_object(self.robot_name)
        if self.robot is None:
            raise RuntimeError(f"[CustomMobileRig] scene object not found: {self.robot_name}")
        return self

    def initialize(self):
        if self.robot is None and self.world is not None:
            self.robot = self.world.scene.get_object(self.robot_name)
        if self.robot is None:
            raise RuntimeError("[CustomMobileRig] robot object is None. add_to_scene()/attach_existing() first.")

        self.robot.initialize()
        self.refresh_dof_maps()
        return self

    def refresh_dof_maps(self):
        self.dof_names = [str(x) for x in self.robot.dof_names]
        self.base_idx = [self.robot.get_dof_index(n) for n in self.base_joint_names]
        all_idx = list(range(len(self.dof_names)))
        base_set = set(int(i) for i in self.base_idx)
        self.other_idx = [i for i in all_idx if i not in base_set]
        self.arm_idx = [i for i, n in enumerate(self.dof_names) if n.startswith("ur_arm_")]
        return self

    # ---------- state snapshots ----------
    def get_q(self) -> np.ndarray:
        return np.array(self.robot.get_joint_positions(), dtype=np.float64)

    def get_qd(self) -> np.ndarray:
        return np.array(self.robot.get_joint_velocities(), dtype=np.float32)

    def set_q(self, q):
        self.robot.set_joint_positions(np.array(q, dtype=np.float32))

    def set_qd(self, qd):
        self.robot.set_joint_velocities(np.array(qd, dtype=np.float32))

    def snapshot_holds(self):
        q0 = self.get_q()
        self.q_hold_other = q0.copy()
        self.base_q_lock = np.array([q0[i] for i in self.base_idx], dtype=np.float64)
        return q0

    def zero_all_velocities(self):
        try:
            qd = self.get_qd()
            qd[:] = 0.0
            self.set_qd(qd)
        except Exception:
            pass

    # ---------- base helpers ----------
    def get_base_dummy_q(self):
        q = self.get_q()
        return np.array([q[i] for i in self.base_idx], dtype=np.float64)

    def set_base_dummy_q_with_hold(self, qx: float, qy: float, qyaw: float):
        if self.q_hold_other is None:
            self.snapshot_holds()

        q_cmd = np.array(self.q_hold_other, dtype=np.float64)
        ix, iy, iyaw = self.base_idx
        q_cmd[ix] = float(qx)
        q_cmd[iy] = float(qy)
        q_cmd[iyaw] = float(qyaw)
        self.set_q(q_cmd)
        return q_cmd

    def apply_base_lock(self):
        if self.base_idx is None or self.base_q_lock is None:
            return

        # position lock
        try:
            q = self.get_q()
            for k, idx in enumerate(self.base_idx):
                q[idx] = self.base_q_lock[k]
            self.set_q(q)
        except Exception:
            pass

        # velocity lock
        try:
            qd = self.get_qd()
            qd[self.base_idx] = 0.0
            self.set_qd(qd)
        except Exception:
            pass

    def __repr__(self):
        return f"CustomMobileRig(name={self.robot_name!r}, prim={self.robot_prim_path!r})"
