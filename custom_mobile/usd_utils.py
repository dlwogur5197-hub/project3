import numpy as np
from pxr import Usd, UsdGeom
import omni.usd

from .compat import get_core_classes


def xform_world_pose(stage, prim_path: str):
    """
    Returns (position[np.ndarray shape(3)], GfMatrix4d) or (None, None)
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None, None

    img = UsdGeom.Imageable(prim)
    if not img:
        return None, None

    m = img.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    p = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
    return p, m


def xform_world_pos(stage, prim_path: str):
    p, _ = xform_world_pose(stage, prim_path)
    return p


def xform_world_quat_xyzw(stage, prim_path: str):
    p, m = xform_world_pose(stage, prim_path)
    if m is None:
        return None
    try:
        q = m.ExtractRotationQuat()  # GfQuatd
    except Exception:
        q = m.ExtractRotation().GetQuat()
    real = float(q.GetReal())
    imag = q.GetImaginary()
    return np.array([float(imag[0]), float(imag[1]), float(imag[2]), real], dtype=np.float64)


def ensure_target_cube(world, prim_path="/World/target_cube",
                       position=(0.8, 0.0, 1.0),
                       scale=(0.06, 0.06, 0.06),
                       color=(1.0, 0.2, 0.2)):
    """
    Creates a visible cube target only if the prim does not already exist.
    Returns prim_path.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        return prim_path

    _, _, VisualCuboid = get_core_classes()

    try:
        world.scene.add(
            VisualCuboid(
                prim_path=prim_path,
                name=prim_path.split("/")[-1] or "target_cube",
                position=np.array(position, dtype=np.float32),
                scale=np.array(scale, dtype=np.float32),
                color=np.array(color, dtype=np.float32),
            )
        )
        return prim_path
    except Exception:
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(tuple(float(x) for x in position))
        xf.AddScaleOp().Set(tuple(float(x) for x in scale))
        return prim_path
