import math
import time
import numpy as np

from .params import MoveBaseSquareParams


class BaseSquareDriver:
    """
    Dummy-base square motion driver (position write) + hold all non-base DOFs.

    Usage:
        driver = BaseSquareDriver(params)
        driver.bind(rig)
        ...
        driver.step(dt)
    """

    def __init__(self, params: MoveBaseSquareParams | None = None):
        self.params = params or MoveBaseSquareParams()
        self.rig = None

        self._segments = []
        self._seg_i = 0
        self._last_dbg_t = 0.0
        self._initialized = False

    def bind(self, rig):
        self.rig = rig
        q0 = self.rig.snapshot_holds()  # captures non-base hold + base lock snapshot
        self.rig.zero_all_velocities()
        self._build_square_segments_from_current_q()
        self._initialized = True
        return self

    def reset(self):
        self._segments = []
        self._seg_i = 0
        self._last_dbg_t = 0.0
        self._initialized = False

    def _build_square_segments_from_current_q(self):
        qx, qy, qyaw = [float(v) for v in self.rig.get_base_dummy_q()]
        s = float(self.params.side)

        # segment format:
        # ("line", target_qx, target_qy)
        # ("yaw", target_qyaw)
        self._segments = [
            ("line", qx + s, qy),
            ("yaw", qyaw + math.pi / 2.0),
            ("line", qx + s, qy + s),
            ("yaw", qyaw + math.pi),
            ("line", qx, qy + s),
            ("yaw", qyaw + 3.0 * math.pi / 2.0),
            ("line", qx, qy),
            ("yaw", qyaw + 2.0 * math.pi),
        ]
        self._seg_i = 0

    def step(self, dt: float | None):
        if not self._initialized or self.rig is None:
            return

        dt = float(dt) if (dt is not None and dt > 0) else (1.0 / 60.0)

        qx, qy, qyaw = [float(v) for v in self.rig.get_base_dummy_q()]
        mode, a, *rest = self._segments[self._seg_i]

        lin_rate = float(self.params.lin_rate)
        yaw_rate = float(self.params.yaw_rate)

        qx_cmd, qy_cmd, qyaw_cmd = qx, qy, qyaw

        if mode == "line":
            tx, ty = float(a), float(rest[0])
            dx = tx - qx
            dy = ty - qy
            dist = math.hypot(dx, dy)

            if dist <= self.params.pos_tol:
                qx_cmd, qy_cmd = tx, ty
                self._advance_segment()
            else:
                step_len = min(lin_rate * dt, dist)
                if dist > 1e-12:
                    qx_cmd += step_len * dx / dist
                    qy_cmd += step_len * dy / dist

        elif mode == "yaw":
            tyaw = float(a)
            err = tyaw - qyaw

            if abs(err) <= self.params.yaw_tol:
                qyaw_cmd = tyaw
                self._advance_segment()
            else:
                d = max(-yaw_rate * dt, min(yaw_rate * dt, err))
                qyaw_cmd += d

        self.rig.set_base_dummy_q_with_hold(qx_cmd, qy_cmd, qyaw_cmd)

        now = time.time()
        if now - self._last_dbg_t > self.params.debug_period:
            self._last_dbg_t = now
            print(
                "[BaseSquareDriver] "
                f"seg={self._seg_i+1}/{len(self._segments)} mode={self._segments[self._seg_i][0]} "
                f"dummy_q=({qx_cmd:.3f},{qy_cmd:.3f},{qyaw_cmd:.3f})"
            )

    def _advance_segment(self):
        self._seg_i += 1
        if self._seg_i >= len(self._segments):
            if self.params.repeat:
                self._build_square_segments_from_current_q()
            else:
                self._seg_i = len(self._segments) - 1
