"""
Isaac Sim compatibility imports (5.0 + legacy omni.isaac fallback).
Import this from project scripts/modules to avoid repeating try/except blocks.
"""

def get_core_classes():
    try:
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.api.objects import VisualCuboid
    except Exception:
        from omni.isaac.core.prims import SingleArticulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.objects import VisualCuboid
    return SingleArticulation, add_reference_to_stage, VisualCuboid


def get_motion_generation():
    try:
        import isaacsim.robot_motion.motion_generation as mg
    except Exception:
        import omni.isaac.motion_generation as mg
    return mg
