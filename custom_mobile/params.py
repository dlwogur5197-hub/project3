from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class MoveBaseSquareParams:
    robot_prim_path: str = "/ridgeback"
    robot_name: str = "custom_mobile_ur5"
    side: float = 4.0
    repeat: bool = True
    lin_rate: float = 0.45      # dummy q-space m-ish per sec (prismatic q units)
    yaw_rate: float = 0.90      # rad/s in dummy yaw q
    pos_tol: float = 0.01
    yaw_tol: float = 0.02
    base_joint_names: Tuple[str, str, str] = (
        "dummy_base_prismatic_x_joint",
        "dummy_base_prismatic_y_joint",
        "dummy_base_revolute_z_joint",
    )
    debug_period: float = 1.0


@dataclass
class RmpArmFollowParams:
    robot_prim_path: str = "/ridgeback"
    robot_name: str = "custom_mobile_ur5"
    target_prim_path: str = "/World/target_cube"
    ee_debug_prim_path: str = "/ridgeback/ur_arm_wrist_3_link"
    ur_arm_base_link_prim_path: str = "/ridgeback/ur_arm_base_link"
    scale_comp: float = 1.0 / 2.5
    z_bias: float = -0.04
    base_lock_after_action: bool = False
    base_joint_names: Tuple[str, str, str] = (
        "dummy_base_prismatic_x_joint",
        "dummy_base_prismatic_y_joint",
        "dummy_base_revolute_z_joint",
    )
    physics_dt_default: float = 1.0 / 60.0
    debug_period: float = 1.0
    create_target_if_missing: bool = True
    target_init_pos: Tuple[float, float, float] = (0.8, 0.0, 1.0)
    target_init_scale: Tuple[float, float, float] = (0.06, 0.06, 0.06)
