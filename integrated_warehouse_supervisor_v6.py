#!/usr/bin/env python3
"""
Isaac Sim + ROS2 Integrated Warehouse Supervisor (Smooth Manipulation & Auto-Normal)
- [모션 개선] 로봇 팔 순간이동 현상 수정 (시간에 따른 선형 보간 스무스 모션 적용)
- [자동화] 가스 농도(ppm)가 0에 도달하면 자동으로 Normal(상황 종료) 단계로 넘어가도록 개선
- 덤프 분석 기반 Action Graph (FSM, 셔터, 밸브, 가스) 100% 동기화
"""

import sys
import argparse
import math
import time
import random
import threading
from dataclasses import dataclass
from pathlib import Path
from collections import deque
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String, Float32, Int32, Bool

# =============================================================
# 💡 0. 파라미터 파싱 및 초기화
# =============================================================
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--headless", action="store_true")
    p.add_argument("--map-yaml", type=str, required=True)
    p.add_argument("--env-usd", type=str, default="/home/rokey/Downloads/basic12.usd")
    p.add_argument("--use-default-ground", action="store_true")

    p.add_argument("--topic-gas-start", type=str, default="/main/gas_start")
    p.add_argument("--topic-normal", type=str, default="/main/normal")
    p.add_argument("--topic-gas-location", type=str, default="/main/gas_location")
    p.add_argument("--topic-mobile-done", type=str, default="/main/mobile_done")
    p.add_argument("--topic-mobile-arrived", type=str, default="/main/mobile_arrived")

    p.add_argument("--patrol-lin", type=float, default=2.55)
    p.add_argument("--patrol-yaw", type=float, default=0.90)
    p.add_argument("--patrol-pos-tol", type=float, default=0.02)
    p.add_argument("--patrol-yaw-tol", type=float, default=0.05)
    p.add_argument("--patrol-seed", type=int, default=7)
    p.add_argument("--no-backtrack", action="store_true", default=True)
    p.add_argument("--patrol-points", type=str, default="-1.1,0.0;-1.1,28.0;-8.7,28.0;-8.7,0.0;-13.2,0.0;-13.2,28.0;-20.8,28.0;-20.8,0.0")
    
    p.add_argument("--max-speed", type=float, default=5.00)
    p.add_argument("--yaw-rate", type=float, default=1.30)
    p.add_argument("--body-radius", type=float, default=1.0)
    p.add_argument("--safety-margin", type=float, default=0.1)
    p.add_argument("--lookahead", type=float, default=4.0)
    p.add_argument("--front-stop", type=float, default=1.80)
    p.add_argument("--front-slow", type=float, default=3.20)
    p.add_argument("--align-thresh-deg", type=float, default=18.0)
    p.add_argument("--candidate-step-deg", type=float, default=15.0)
    p.add_argument("--candidate-span-deg", type=float, default=150.0)
    p.add_argument("--free-unknown", action="store_true")
    p.add_argument("--world-x-bias", type=float, default=0.0)
    p.add_argument("--world-y-bias", type=float, default=0.0)
    p.add_argument("--yaw-bias-deg", type=float, default=0.0)
    p.add_argument("--planner", type=str, default="astar", choices=["astar", "none"])
    p.add_argument("--replan-period", type=float, default=3.0)
    p.add_argument("--waypoint-lookahead", type=float, default=3.0)
    p.add_argument("--goal-progress-timeout", type=float, default=6.0)
    p.add_argument("--goal-tol", type=float, default=0.1) 
    return p.parse_args()

ARGS = parse_args()
THIS_FILE = Path(__file__).resolve()
PROJECT_DIR = THIS_FILE.parent
if str(PROJECT_DIR) not in sys.path: sys.path.insert(0, str(PROJECT_DIR))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": bool(ARGS.headless)})

try:
    from isaacsim.core.utils.extensions import enable_extension
except ImportError:
    from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.graph.action")
enable_extension("isaacsim.ros2.bridge")

print("[System] 확장 프로그램 로딩 대기 중...")
for _ in range(20): 
    simulation_app.update()
print("[System] 확장 프로그램 로딩 완료!")

try: from isaacsim.core.api import World
except Exception: from omni.isaac.core import World

import omni.usd
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
import omni.replicator.core as rep
from custom_mobile import resolve_custom_mobile_usd, CustomMobileRig
from pyzbar.pyzbar import decode

# =============================================================
# 💡 1. 가스 시뮬레이터
# =============================================================
class GasSimulator:
    def __init__(self):
        self.base_ppm = 10.0
        self.ramp_down_s = 8.0
        self.zones = ['A', 'B', 'C', 'D']
        self.ppm = {z: self.base_ppm for z in self.zones}
        self.active_leak = {z: False for z in self.zones}
        self.ramp_active = {z: False for z in self.zones}
        self.ramp_start_ppm = {z: 0.0 for z in self.zones}
        self.ramp_start_t = {z: 0.0 for z in self.zones}

    def trigger_leak(self, z):
        self.active_leak[z] = True
        self.ramp_active[z] = False

    def start_ramp_down(self, z, current_time):
        if self.active_leak[z] or self.ppm[z] > self.base_ppm:
            self.active_leak[z] = False
            self.ramp_active[z] = True
            self.ramp_start_ppm[z] = self.ppm[z]
            self.ramp_start_t[z] = current_time

    def update(self, current_time, dt):
        for z in self.zones:
            if self.ramp_active[z]:
                elapsed = current_time - self.ramp_start_t[z]
                if elapsed >= self.ramp_down_s:
                    self.ppm[z] = 0.0
                else:
                    alpha = 1.0 - (elapsed / self.ramp_down_s)
                    self.ppm[z] = max(0.0, self.ramp_start_ppm[z] * alpha)
            elif self.active_leak[z]:
                self.ppm[z] = min(2000.0, self.ppm[z] + 200.0 * dt)

    def reset(self, z):
        self.active_leak[z] = False
        self.ramp_active[z] = False
        self.ppm[z] = self.base_ppm

# =============================================================
# 💡 2. ROS2 통신 구조 및 이벤트 큐
# =============================================================
class EventQueue:
    def __init__(self):
        self._dq = deque(); self._lock = threading.Lock()
    def push(self, ev):
        with self._lock: self._dq.append(ev)
    def pop_all(self):
        with self._lock:
            out = list(self._dq); self._dq.clear(); return out

@dataclass
class EvGasStart: code: str
@dataclass
class EvNormal: pass
@dataclass
class EvGasLocation: x: float; y: float; yaw: float
@dataclass
class EvIsolationDone: code: str

DISPATCH_POINTS = {
    "A": (-18.0, 28.1, 1.57), "B": (-3.0, 28.1, 1.57),
    "C": (-24.2, -5.41, 3.14), "D": (2.96, -5.41, 0),
}
DRONE_COMMAND_TARGETS = {
    'A': np.array([-18.0, 29.0, 7.0]), 'B': np.array([-3.0, 29.0, 7.0]),
    'C': np.array([-25.5, -5.41, 7.0]), 'D': np.array([4.0, -5.41, 7.0])
}

SHUTTER_MAPPING = {'A': [1, 4], 'B': [1, 2], 'C': [3, 4], 'D': [2, 3]}
VALVE_OPEN = {'A': 90.0, 'B': 90.0, 'C': 0.0, 'D': 0.0}
VALVE_CLOSE = {'A': 0.0, 'B': 0.0, 'C': 90.0, 'D': 90.0}

global_ros_node = None

class UnifiedROSBridge(Node):
    def __init__(self, eq):
        super().__init__("integrated_warehouse_supervisor")
        self.eq = eq
        
        self.create_subscription(String, ARGS.topic_gas_start, self._on_gas_start, 10)
        self.create_subscription(Empty, ARGS.topic_normal, self._on_normal, 10)
        self.create_subscription(String, ARGS.topic_gas_location, self._on_gas_location, 10)

        self.create_subscription(Bool, '/incident/isolation_done_a', lambda msg: self._on_isolation('A', msg.data), 10)
        self.create_subscription(Bool, '/incident/isolation_done_b', lambda msg: self._on_isolation('B', msg.data), 10)
        self.create_subscription(Bool, '/incident/isolation_done_c', lambda msg: self._on_isolation('C', msg.data), 10)
        self.create_subscription(Bool, '/incident/isolation_done_d', lambda msg: self._on_isolation('D', msg.data), 10)

        self.gas_loc_pub = self.create_publisher(String, ARGS.topic_gas_location, 10)
        self.mobile_done_pub = self.create_publisher(Empty, ARGS.topic_mobile_done, 10)
        self.mobile_arrived_pub = self.create_publisher(Empty, ARGS.topic_mobile_arrived, 10)
        self.barcode_pub = self.create_publisher(String, '/drone/barcode_data', 10)
        self.gas_pub = self.create_publisher(String, '/drone/gas_ppm', 10)

        self.shutter_pubs = {i: self.create_publisher(Bool, f'/hazard/shutter_cmd_{i}', 10) for i in range(1, 5)}
        self.valve_pubs = {z: self.create_publisher(Float32, f'/valve/{z.lower()}_angle_deg', 10) for z in ['A','B','C','D']}
        self.gas_ag_pubs = {z: self.create_publisher(Float32, f'/hazmat/gas_fixed_{z.lower()}_ppm', 10) for z in ['A','B','C','D']}
        self.fsm_pubs = {z: self.create_publisher(Int32, f'/hazard/fsm_state_code_{z.lower()}', 10) for z in ['A','B','C','D']}

    def _on_gas_start(self, msg): 
        random_zone = random.choice(['A', 'B', 'C', 'D'])
        print(f"\n[System] ⚠️ 랜덤 가스 누출 구역 확정: '{random_zone}' 구역")
        self.eq.push(EvGasStart(code=random_zone))
        
    def _on_normal(self, _msg): self.eq.push(EvNormal())
    def _on_gas_location(self, msg):
        try:
            parts = msg.data.split()
            self.eq.push(EvGasLocation(x=float(parts[0]), y=float(parts[1]), yaw=float(parts[2])))
        except Exception: pass
    
    def _on_isolation(self, code, is_done):
        if is_done: self.eq.push(EvIsolationDone(code=code))

    def publish_master_state(self, fsm_state, shutter_state, valve_state):
        for k, v in fsm_state.items():
            msg = Int32(); msg.data = int(v); self.fsm_pubs[k].publish(msg)
        for k, v in shutter_state.items():
            msg = Bool(); msg.data = bool(v); self.shutter_pubs[k].publish(msg)
        for k, v in valve_state.items():
            msg = Float32(); msg.data = float(v); self.valve_pubs[k].publish(msg)

def start_ros(eq: EventQueue):
    global global_ros_node
    def _spin():
        global global_ros_node
        rclpy.init(); global_ros_node = UnifiedROSBridge(eq)
        try: rclpy.spin(global_ros_node)
        finally:
            try: global_ros_node.destroy_node()
            except Exception: pass
            rclpy.shutdown()
    th = threading.Thread(target=_spin, daemon=True); th.start(); return th

# =============================================================
# 💡 3. Mobile Robot Map & Reactive Navigation (생략 없음)
# =============================================================
import heapq
@dataclass
class MapMeta:
    image_path: str; resolution: float; origin_x: float; origin_y: float; origin_yaw: float
    negate: int; occupied_thresh: float; free_thresh: float

class OccupancyMap2D:
    def __init__(self, yaml_path: str, treat_unknown_as_occupied: bool = True):
        self.yaml_path = str(Path(yaml_path).expanduser().resolve()); self.meta = self._load_yaml(self.yaml_path)
        self.gray = self._load_image_gray(self.meta.image_path); self.h, self.w = self.gray.shape[:2]
        self.treat_unknown_as_occupied = bool(treat_unknown_as_occupied)
        self.occ_raw = self._build_occ_mask(self.gray, self.meta); self.occ_inflated = self.occ_raw.copy(); self._free_cache = None
    def _load_yaml(self, yaml_path: str) -> MapMeta:
        import yaml
        with open(yaml_path, "r", encoding="utf-8") as f: data = yaml.safe_load(f)
        img_path = str((Path(yaml_path).parent / data["image"]).resolve())
        ox, oy, oyaw = [float(v) for v in data.get("origin", [0.0, 0.0, 0.0])]
        return MapMeta(img_path, float(data.get("resolution", 0.05)), ox, oy, oyaw, int(data.get("negate", 0)), float(data.get("occupied_thresh", 0.65)), float(data.get("free_thresh", 0.196)))
    def _load_image_gray(self, image_path: str) -> np.ndarray:
        from PIL import Image
        img = Image.open(image_path)
        if img.mode == "RGBA": bg = Image.new("RGBA", img.size, (255, 255, 255, 255)); img = Image.alpha_composite(bg, img).convert("L")
        else: img = img.convert("L")
        return np.array(img, dtype=np.uint8)
    def _build_occ_mask(self, gray: np.ndarray, meta: MapMeta) -> np.ndarray:
        g = gray.astype(np.float32) / 255.0; occ_prob = (1.0 - g) if int(meta.negate) == 0 else g
        occupied = occ_prob >= float(meta.occupied_thresh); free = occ_prob <= float(meta.free_thresh); unknown = ~(occupied | free)
        return ((occupied | unknown) if self.treat_unknown_as_occupied else occupied).astype(bool)
    def inflate(self, radius_m: float):
        rad_px = max(0, int(math.ceil(float(radius_m) / self.meta.resolution)))
        if rad_px <= 0: self.occ_inflated = self.occ_raw.copy(); return
        occ = self.occ_raw; H, W = occ.shape; out = occ.copy()
        yy, xx = np.ogrid[-rad_px:rad_px + 1, -rad_px:rad_px + 1]
        offsets = np.argwhere((xx * xx + yy * yy) <= (rad_px * rad_px)) - rad_px
        for dy, dx in offsets:
            out[max(0, dy):min(H, H + dy), max(0, dx):min(W, W + dx)] |= occ[max(0, -dy):min(H, H - dy), max(0, -dx):min(W, W - dx)]
        self.occ_inflated = out
    def world_to_pixel(self, x: float, y: float): return (self.h - 1) - int(math.floor((float(y) - self.meta.origin_y) / self.meta.resolution)), int(math.floor((float(x) - self.meta.origin_x) / self.meta.resolution))
    def pixel_to_world_center(self, row: int, col: int): return self.meta.origin_x + (float(col) + 0.5) * self.meta.resolution, self.meta.origin_y + (float((self.h - 1) - row) + 0.5) * self.meta.resolution
    def in_bounds_px(self, row: int, col: int) -> bool: return 0 <= row < self.h and 0 <= col < self.w
    def is_blocked_world(self, x: float, y: float, inflated: bool = True) -> bool:
        row, col = self.world_to_pixel(x, y)
        return not self.in_bounds_px(row, col) or bool((self.occ_inflated if inflated else self.occ_raw)[row, col])
    def raycast_free_distance(self, x: float, y: float, yaw: float, max_range_m: float, step_m: float | None = None, inflated: bool = True) -> float:
        step_m = step_m or max(0.01, 0.5 * self.meta.resolution); d = 0.0; c = math.cos(yaw); s = math.sin(yaw)
        if self.is_blocked_world(x, y, inflated=inflated): return 0.0
        while d < max_range_m:
            d += step_m
            if self.is_blocked_world(x + c * d, y + s * d, inflated=inflated): return max(0.0, d - step_m)
        return float(max_range_m)
    def nearest_free_heading_score(self, x: float, y: float, headings: np.ndarray, lookahead: float) -> np.ndarray:
        scores = np.zeros(len(headings), dtype=np.float64)
        for i, th in enumerate(headings):
            d0 = self.raycast_free_distance(x, y, float(th), lookahead, inflated=True)
            dL = self.raycast_free_distance(x, y, float(th) + math.radians(25), min(lookahead, 1.6), inflated=True)
            dR = self.raycast_free_distance(x, y, float(th) - math.radians(25), min(lookahead, 1.6), inflated=True)
            scores[i] = 1.8 * d0 + 0.5 * min(dL, dR) + 0.2 * (dL + dR)
        return scores
    def _nearest_free_cell(self, row: int, col: int, max_radius_px: int = 60, inflated: bool = True):
        grid = self.occ_inflated if inflated else self.occ_raw
        if self.in_bounds_px(row, col) and not grid[row, col]: return int(row), int(col)
        for r in range(1, max_radius_px + 1):
            for rr in (max(0, row - r), min(self.h - 1, row + r)):
                for cc in range(max(0, col - r), min(self.w - 1, col + r) + 1):
                    if self.in_bounds_px(rr, cc) and not grid[rr, cc]: return int(rr), int(cc)
            for cc in (max(0, col - r), min(self.w - 1, col + r)):
                for rr in range(max(0, row - r) + 1, min(self.h - 1, row + r)):
                    if self.in_bounds_px(rr, cc) and not grid[rr, cc]: return int(rr), int(cc)
        return None
    def astar_path_world(self, sx: float, sy: float, gx: float, gy: float):
        s = self._nearest_free_cell(*self.world_to_pixel(sx, sy), inflated=True); g = self._nearest_free_cell(*self.world_to_pixel(gx, gy), inflated=True)
        if s is None or g is None: return []
        sr, sc = s; gr, gc = g
        if (sr, sc) == (gr, gc): return [(float(gx), float(gy))]
        grid = self.occ_inflated; H, W = grid.shape
        neigh = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        open_heap = []; heapq.heappush(open_heap, (math.hypot(sr - gr, sc - gc), 0.0, sr, sc))
        came = {}; gscore = {(sr, sc): 0.0}; closed = set(); expanded = 0
        while open_heap:
            _, cur_g, r, c = heapq.heappop(open_heap)
            if (r, c) in closed: continue
            closed.add((r, c)); expanded += 1
            if (r, c) == (gr, gc):
                path_px = [(r, c)]
                while (r, c) in came: r, c = came[(r, c)]; path_px.append((r, c))
                path_px.reverse(); pts = []; stride = max(1, int(round(0.35 / max(1e-6, self.meta.resolution))))
                for i, (rr, cc) in enumerate(path_px):
                    if i == 0 or i == len(path_px)-1 or (i % stride == 0): pts.append(self.pixel_to_world_center(rr, cc))
                if not pts or math.hypot(pts[-1][0]-gx, pts[-1][1]-gy) > 1e-6: pts.append((float(gx), float(gy)))
                return [(float(px), float(py)) for px, py in pts]
            if expanded > (int(H * W * 0.35) if H * W > 0 else 50000): break
            for dr, dc, w in neigh:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and not grid[rr, cc] and not (dr != 0 and dc != 0 and (grid[r, cc] or grid[rr, c])):
                    ng = cur_g + w; key = (rr, cc)
                    if ng < gscore.get(key, float("inf")):
                        gscore[key] = ng; came[key] = (r, c); heapq.heappush(open_heap, (ng + math.hypot(rr - gr, cc - gc), ng, rr, cc))
        return []

def wrap_pi(a: float) -> float: return (a + math.pi) % (2.0 * math.pi) - math.pi

@dataclass
class ReactiveRoamParams:
    max_speed: float; yaw_rate: float; front_stop: float; front_slow: float
    lookahead: float; align_thresh: float; candidate_step: float; candidate_span: float
    world_x_bias: float; world_y_bias: float; yaw_bias: float; goal_tol: float
    planner: str; replan_period: float; waypoint_lookahead: float; goal_progress_timeout: float; seed: int = 42

class ReactiveMapRoamDriver:
    def __init__(self, occ_map: OccupancyMap2D, params: ReactiveRoamParams):
        self.map = occ_map; self.params = params; self.rig = None; self._rng = np.random.default_rng(int(params.seed))
        self._desired_heading = None; self._mode = "roam"; self._recover_timer = 0.0
        self._last_q = None; self._stuck_timer = 0.0; self._path = []; self._path_i = 0; self._last_replan_t = -1e9
        self._last_progress_t = time.time(); self._best_goal_dist = float("inf")
        self._goal = None; self._goal_yaw = None
    
    def bind(self, rig: CustomMobileRig):
        self.rig = rig; self.rig.snapshot_holds(); self.rig.zero_all_velocities(); self._last_q = self.rig.get_base_dummy_q().copy(); return self
    
    def set_goal(self, gx: float, gy: float, gyaw: float = None):
        self._goal = (float(gx), float(gy)); self._goal_yaw = gyaw; self._mode = "roam"
        self._recover_timer = 0.0; self._stuck_timer = 0.0
        self._path = []; self._path_i = 0; self._last_replan_t = -1e9; self._last_progress_t = time.time(); self._best_goal_dist = float("inf")
    
    def clear_goal(self): self._goal = None; self._goal_yaw = None; self._mode = "roam"; self._path = []; self._path_i = 0
    
    def _get_pose_for_map(self):
        qx, qy, qyaw = [float(v) for v in self.rig.get_base_dummy_q()]
        return qx, qy, qyaw, qx + self.params.world_x_bias, qy + self.params.world_y_bias, qyaw + self.params.yaw_bias
    
    def _candidate_headings(self, yaw: float) -> np.ndarray:
        span = self.params.candidate_span; step = self.params.candidate_step
        offs = np.linspace(-span, span, int(round((2.0 * span) / step)) + 1)
        extra = np.array([-math.pi, -math.pi/2.0, 0.0, math.pi/2.0, math.pi])
        return np.array([wrap_pi(yaw + d) for d in np.concatenate([offs, extra])], dtype=np.float64)
    
    def _choose_heading(self, x: float, y: float, yaw: float, goal_heading: float | None, dynamic_front_stop: float):
        hs = self._candidate_headings(yaw); free_scores = self.map.nearest_free_heading_score(x, y, hs, self.params.lookahead)
        turn_pen = np.array([abs(wrap_pi(h - yaw)) for h in hs], dtype=np.float64)
        persist_pen = np.zeros_like(turn_pen) if self._desired_heading is None else np.array([abs(wrap_pi(h - self._desired_heading)) for h in hs], dtype=np.float64)
        cand_front = np.array([self.map.raycast_free_distance(x, y, h, max(0.8, dynamic_front_stop + 0.2), inflated=True) for h in hs])
        valid = cand_front >= (dynamic_front_stop * 0.85)
        total = free_scores - 0.35 * turn_pen - 0.15 * persist_pen + self._rng.normal(0.0, 0.01, size=len(hs))
        if goal_heading is not None: total -= 0.75 * np.array([abs(wrap_pi(h - goal_heading)) for h in hs], dtype=np.float64)
        total[~valid] -= 10.0; idx = int(np.argmax(total))
        return float(hs[idx]), {"front_d": float(self.map.raycast_free_distance(x, y, yaw, self.params.lookahead, inflated=True)), "valid_count": int(np.count_nonzero(valid))}
    
    def is_goal_reached(self) -> bool:
        return self._mode == "goal_reached"

    def step(self, dt: float):
        dt = float(dt) if dt > 0 else (1.0/60.0)
        qx, qy, qyaw, x, y, yaw = self._get_pose_for_map(); now = time.time(); active_goal = final_goal = final_dist = None
        
        if self._goal is not None:
            if self.params.planner == "none": self._path = [self._goal]; self._path_i = 0; self._last_replan_t = now
            elif (now - self._last_replan_t) >= self.params.replan_period or not self._path:
                self._path = self.map.astar_path_world(x, y, self._goal[0], self._goal[1]); self._path_i = 0; self._last_replan_t = now
            final_goal = self._goal; final_dist = math.hypot(final_goal[0] - x, final_goal[1] - y)
            if self._path:
                while self._path_i + 1 < len(self._path) and math.hypot(self._path[self._path_i+1][0]-x, self._path[self._path_i+1][1]-y) <= math.hypot(self._path[self._path_i][0]-x, self._path[self._path_i][1]-y): self._path_i += 1
                idx = self._path_i
                while idx + 1 < len(self._path) and math.hypot(self._path[idx][0]-x, self._path[idx][1]-y) < self.params.waypoint_lookahead: idx += 1
                active_goal = self._path[min(idx, len(self._path)-1)]
            if final_dist + 1e-6 < self._best_goal_dist: self._best_goal_dist = final_dist; self._last_progress_t = now
            elif (now - self._last_progress_t) > self.params.goal_progress_timeout:
                self._path = self.map.astar_path_world(x, y, self._goal[0], self._goal[1]); self._path_i = 0; self._last_replan_t = now; self._last_progress_t = now; self._best_goal_dist = final_dist
        
        q_now = np.array([qx, qy, qyaw], dtype=np.float64)
        if self._last_q is None: self._last_q = q_now.copy()
        dq_xy = float(np.linalg.norm(q_now[:2] - self._last_q[:2])); self._last_q = q_now.copy()

        dynamic_front_stop = self.params.front_stop
        in_blocked = self.map.is_blocked_world(x, y, inflated=True)
        if final_dist is not None and final_dist < 0.8: 
            dynamic_front_stop = 0.05 
            in_blocked = False        

        chosen, dbg = self._choose_heading(x, y, yaw, math.atan2(active_goal[1]-y, active_goal[0]-x) if active_goal is not None else None, dynamic_front_stop)
        self._desired_heading = chosen; yaw_err = wrap_pi(chosen - yaw); front_d = dbg["front_d"]
        vx_cmd = vy_cmd = w_cmd = 0.0

        if final_dist is not None and final_dist <= self.params.goal_tol:
            if self._mode not in ["align_goal", "goal_reached"]:
                if self._goal_yaw is not None:
                    self._mode = "align_goal"
                else:
                    self._mode = "goal_reached"

        if self._mode == "goal_reached": pass
        elif self._mode == "align_goal":
            yaw_err = wrap_pi(self._goal_yaw - yaw)
            w_cmd = float(np.clip(1.5 * yaw_err, -self.params.yaw_rate, self.params.yaw_rate))
            if abs(yaw_err) <= 0.05: 
                self._mode = "goal_reached"
                w_cmd = 0.0

        elif self._mode == "recover_back":
            self._recover_timer -= dt; vx_cmd = -0.18
            if self._recover_timer <= 0.0:
                self._mode = "recover_turn"; self._recover_timer = 0.8 + 0.7 * float(self._rng.random())
                self._desired_heading = wrap_pi(yaw + (-1.0 if self._rng.random() < 0.5 else 1.0) * math.radians(110.0))
        elif self._mode == "recover_turn":
            self._recover_timer -= dt; yaw_err2 = wrap_pi(self._desired_heading - yaw)
            w_cmd = float(np.clip(2.2 * yaw_err2, -self.params.yaw_rate, self.params.yaw_rate))
            if abs(yaw_err2) < math.radians(10.0) or self._recover_timer <= 0.0: self._mode = "roam"; self._recover_timer = 0.0
        else:
            if in_blocked: w_cmd = self.params.yaw_rate * 0.6
            elif front_d < dynamic_front_stop:
                w_cmd = float(np.clip(2.5 * yaw_err, -self.params.yaw_rate, self.params.yaw_rate))
                if abs(yaw_err) < math.radians(12.0) and dbg["valid_count"] <= 2: self._mode = "recover_back"; self._recover_timer = 0.6
            else:
                if abs(yaw_err) > self.params.align_thresh: w_cmd = float(np.clip(2.0 * yaw_err, -self.params.yaw_rate, self.params.yaw_rate))
                else:
                    slow_front = float(np.clip((front_d - dynamic_front_stop) / max(1e-6, (self.params.front_slow - dynamic_front_stop)), 0.15, 1.0))
                    slow_goal = float(np.clip(final_dist / max(1e-6, 1.2), 0.18, 1.0)) if final_dist is not None else 1.0
                    vx_cmd = self.params.max_speed * float(min(slow_front, slow_goal))
                    w_cmd = float(np.clip(1.6 * yaw_err, -self.params.yaw_rate, self.params.yaw_rate))

        if self._mode not in ["goal_reached", "align_goal"] and (abs(vx_cmd) > 0.08 or abs(vy_cmd) > 0.06) and dq_xy < 0.002: self._stuck_timer += dt
        else: self._stuck_timer = max(0.0, self._stuck_timer - 0.5 * dt)
        if self._stuck_timer > 1.2 and self._mode == "roam":
            self._mode = "recover_back"; self._recover_timer = 0.7; self._stuck_timer = 0.0
            if self._goal is not None: self._path = self.map.astar_path_world(x, y, self._goal[0], self._goal[1]); self._path_i = 0; self._last_replan_t = now

        qyaw_cmd = qyaw + float(w_cmd) * dt; c = math.cos(qyaw_cmd); s = math.sin(qyaw_cmd)
        self.rig.set_base_dummy_q_with_hold(qx + float(vx_cmd * c - vy_cmd * s) * dt, qy + float(vx_cmd * s + vy_cmd * c) * dt, qyaw_cmd)

def parse_points(s: str): return [(float(a), float(b)) for item in (s or "").split(";") if item.strip() for a, b in [item.split(",")]]
def build_adjacency(points):
    pts = list(points); adj = {i: set() for i in range(len(pts))}; by_x = {}; by_y = {}
    for i, (x, y) in enumerate(pts): by_x.setdefault(x, []).append((y, i)); by_y.setdefault(y, []).append((x, i))
    for arr in by_x.values():
        arr.sort()
        for k in range(len(arr)-1): adj[arr[k][1]].add(arr[k+1][1]); adj[arr[k+1][1]].add(arr[k][1])
    for arr in by_y.values():
        arr.sort()
        for k in range(len(arr)-1): adj[arr[k][1]].add(arr[k+1][1]); adj[arr[k+1][1]].add(arr[k][1])
    return {k: sorted(list(v)) for k, v in adj.items()}

class PatrolRandomAdjacent:
    def __init__(self, rig: CustomMobileRig, points, adj):
        self.rig = rig; self.points = list(points); self.adj = adj; self.rng = np.random.default_rng(int(ARGS.patrol_seed))
        self._anchor_to_nearest()
    def _anchor_to_nearest(self):
        qx, qy, _ = [float(v) for v in self.rig.get_base_dummy_q()]
        self.cur_i = min(range(len(self.points)), key=lambda i: (self.points[i][0] - qx)**2 + (self.points[i][1] - qy)**2)
        self.prev_i = None; self.tgt_i = None; self.mode = "yaw"; self.des_yaw = None
    def reset_after_return(self): self._anchor_to_nearest()
    def _choose_next(self):
        neigh = list(self.adj[self.cur_i])
        if not neigh: return self.cur_i
        if ARGS.no_backtrack and self.prev_i is not None and len(neigh) >= 2 and self.prev_i in neigh:
            neigh = [n for n in neigh if n != self.prev_i] or neigh
        return int(self.rng.choice(neigh))
    def step(self, dt: float):
        dt = float(dt) if dt > 0 else (1.0/60.0)
        qx, qy, qyaw = [float(v) for v in self.rig.get_base_dummy_q()]
        if self.tgt_i is None: self.tgt_i = self._choose_next(); self.mode = "yaw"; self.des_yaw = None
        tx, ty = self.points[self.tgt_i]
        if self.mode == "yaw":
            if self.des_yaw is None:
                dx = tx - qx; dy = ty - qy
                self.des_yaw = (0.0 if dx >= 0 else math.pi) if abs(dx) >= abs(dy) else (math.pi/2.0 if dy >= 0 else -math.pi/2.0)
            err = wrap_pi(self.des_yaw - qyaw)
            if abs(err) <= ARGS.patrol_yaw_tol: self.rig.set_base_dummy_q_with_hold(qx, qy, self.des_yaw); self.mode = "line"
            else: self.rig.set_base_dummy_q_with_hold(qx, qy, qyaw + float(np.clip(err, -ARGS.patrol_yaw*dt, ARGS.patrol_yaw*dt)))
            return
        dx = tx - qx; dy = ty - qy
        if abs(dx) <= ARGS.patrol_pos_tol and abs(dy) <= ARGS.patrol_pos_tol:
            self.rig.set_base_dummy_q_with_hold(tx, ty, qyaw)
            self.prev_i = self.cur_i; self.cur_i = self.tgt_i; self.tgt_i = None; self.mode = "yaw"; self.des_yaw = None
            return
        step = min(ARGS.patrol_lin * dt, abs(dx) if abs(dx) >= abs(dy) else abs(dy))
        self.rig.set_base_dummy_q_with_hold(qx + (math.copysign(step, dx) if abs(dx) >= abs(dy) else 0), qy + (0 if abs(dx) >= abs(dy) else math.copysign(step, dy)), qyaw)

# =============================================================
# 💡 4. Drone Scan Logic
# =============================================================
DRONE_SCAN_LOCATIONS = [
    {"slot": "move", "pos": np.array([2.7, 9.1, 7.0])},
    {"slot": "A-1", "pos": np.array([2.7, 9.1, 4.1])}, {"slot": "A-2", "pos": np.array([2.7, 13.2, 4.1])},
    {"slot": "A-3", "pos": np.array([2.7, 17.2, 4.1])}, {"slot": "A-4", "pos": np.array([2.7, 21.2, 4.3])},
    {"slot": "B-4", "pos": np.array([-4.4, 21.0, 4.3])}, {"slot": "B-3", "pos": np.array([-4.4, 16.8, 4.1])},
    {"slot": "B-2", "pos": np.array([-4.4, 13.0, 4.1])}, {"slot": "B-1", "pos": np.array([-4.4, 9.0, 4.1])},
    {"slot": "move", "pos": np.array([-4.4, 7.0, 4.1])}, {"slot": "move", "pos": np.array([-7.1, 7.0, 4.1])},
    {"slot": "C-1", "pos": np.array([-7.1, 9.1, 4.1])}, {"slot": "C-2", "pos": np.array([-7.1, 13.2, 4.1])},
    {"slot": "C-3", "pos": np.array([-7.1, 17.1, 4.1])}, {"slot": "C-4", "pos": np.array([-7.1, 21.2, 4.3])},
    {"slot": "D-4", "pos": np.array([-14.5, 21.4, 4.2])}, {"slot": "D-3", "pos": np.array([-14.2, 16.9, 4.2])},
    {"slot": "D-2", "pos": np.array([-14.2, 12.9, 4.2])}, {"slot": "D-1", "pos": np.array([-14.2, 8.9, 4.2])},
    {"slot": "move", "pos": np.array([-14.2, 6.9, 4.2])}, {"slot": "move", "pos": np.array([-16.9, 6.9, 4.2])},
    {"slot": "E-1", "pos": np.array([-17.2, 9.3, 4.2])}, {"slot": "E-2", "pos": np.array([-17.2, 13.3, 4.2])},
    {"slot": "E-3", "pos": np.array([-17.2, 17.3, 4.2])}, {"slot": "E-4", "pos": np.array([-17.2, 21.3, 4.2])},
    {"slot": "F-4", "pos": np.array([-24.3, 21.1, 4.2])}, {"slot": "F-3", "pos": np.array([-24.3, 16.9, 4.2])},
    {"slot": "F-2", "pos": np.array([-24.3, 12.9, 4.2])}, {"slot": "F-1", "pos": np.array([-24.3, 8.9, 4.4])},
    {"slot": "move", "pos": np.array([-24.3, 7.5, 4.2])}, {"slot": "move", "pos": np.array([3.0, 7.5, 4.1])},
]

def scan_and_publish_barcode(annotator, camera_name, slot_name, pub):
    if pub is None: return False
    data = annotator.get_data()
    if data is None: return False
    img = data["data"] if isinstance(data, dict) else data

    if img is not None and img.size > 0:
        img = np.array(img, dtype=np.uint8)
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY) if len(img.shape) == 3 and img.shape[2] == 4 else img
        _, gray_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)
        barcodes = decode(gray_img)
        for barcode in barcodes:
            barcode_text = barcode.data.decode('utf-8')
            print(f"🎉 [{camera_name} 카메라] '{slot_name}' 스캔 성공: {barcode_text}")
            try:
                msg = String(); msg.data = f"{slot_name}:{barcode_text}"; pub.publish(msg)
            except: pass
        if len(barcodes) > 0: return True
    return False

# =============================================================
# 💡 5. Main Integrated System
# =============================================================
def open_stage_and_wait(usd_path: str, timeout_s: float = 60.0):
    ctx = omni.usd.get_context()
    ctx.open_stage(usd_path)
    t0 = time.time()
    while True:
        simulation_app.update()
        if not getattr(ctx, "is_loading", lambda: False)(): break
        if time.time() - t0 > timeout_s: raise RuntimeError(f"Stage open timeout: {usd_path}")

def main():
    occ = OccupancyMap2D(ARGS.map_yaml, treat_unknown_as_occupied=(not ARGS.free_unknown))
    inflation = float(ARGS.body_radius + ARGS.safety_margin)
    occ.inflate(inflation)
    
    if not ARGS.use_default_ground: open_stage_and_wait(str(Path(ARGS.env_usd).expanduser()), timeout_s=80.0)

    world = World(stage_units_in_meters=1.0)
    if ARGS.use_default_ground: world.scene.add_default_ground_plane()

    usd_path = resolve_custom_mobile_usd()
    mobile_rig = CustomMobileRig(); mobile_rig.add_to_scene(world, usd_path)
    
    drone_path = "/World/Drone_Group/Drone_Group/Drone_Group/cf2x"
    if not is_prim_path_valid(drone_path): simulation_app.close(); exit()
    drone = Articulation(prim_path=drone_path, name="cf2x_drone")

    left_camera_path = f"{drone_path}/body/rsd455/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
    right_camera_path = f"{drone_path}/body/rsd455_01/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
    rp_left = rep.create.render_product(left_camera_path, (640, 480)) if is_prim_path_valid(left_camera_path) else None
    rp_right = rep.create.render_product(right_camera_path, (640, 480)) if is_prim_path_valid(right_camera_path) else None
    
    rgb_annot_left = rep.AnnotatorRegistry.get_annotator("rgb")
    if rp_left: rgb_annot_left.attach([rp_left])
    rgb_annot_right = rep.AnnotatorRegistry.get_annotator("rgb")
    if rp_right: rgb_annot_right.attach([rp_right])

    import omni.timeline
    try: 
        world.reset()
        omni.timeline.get_timeline_interface().play()
        
        print("[System] 맵 렌더링 버그 우회: Emission 속성을 초기화합니다...")
        stage = omni.usd.get_context().get_stage()
        count = 0
        for prim in stage.Traverse():
            attr = prim.GetAttribute("inputs:enable_emission")
            if attr.IsValid():
                attr.Set(True); attr.Set(False); attr.Set(True)
                count += 1
        print(f"[System] 총 {count}개의 조명/매테리얼 Emission을 재부팅했습니다.")
        for _ in range(5): simulation_app.update()
            
    except Exception as e: 
        print(f"⚠️ Timeline Play 에러 발생: {e}")

    mobile_rig.attach_existing(world).initialize(); mobile_rig.snapshot_holds(); mobile_rig.zero_all_velocities()
    drone.initialize(); drone.post_reset()

    mobile_home_q = mobile_rig.get_q().copy()

    nav_params = ReactiveRoamParams(
        max_speed=float(ARGS.max_speed), yaw_rate=float(ARGS.yaw_rate), front_stop=float(ARGS.front_stop),
        front_slow=float(ARGS.front_slow), lookahead=float(ARGS.lookahead), align_thresh=math.radians(float(ARGS.align_thresh_deg)),
        candidate_step=math.radians(float(ARGS.candidate_step_deg)), candidate_span=math.radians(float(ARGS.candidate_span_deg)),
        world_x_bias=float(ARGS.world_x_bias), world_y_bias=float(ARGS.world_y_bias), yaw_bias=math.radians(float(ARGS.yaw_bias_deg)),
        goal_tol=float(ARGS.goal_tol), planner=str(ARGS.planner), replan_period=float(ARGS.replan_period),
        waypoint_lookahead=float(ARGS.waypoint_lookahead), goal_progress_timeout=float(ARGS.goal_progress_timeout), seed=42,
    )
    mobile_nav = ReactiveMapRoamDriver(occ, nav_params).bind(mobile_rig)
    patrol_pts = parse_points(ARGS.patrol_points)
    mobile_patrol = PatrolRandomAdjacent(mobile_rig, patrol_pts, build_adjacency(patrol_pts))

    eq = EventQueue()
    _ros_th = start_ros(eq)
    gas_sim = GasSimulator()

    master_fsm_state = {'A': 4, 'B': 4, 'C': 4, 'D': 4} 
    master_shutter_state = {1: False, 2: False, 3: False, 4: False} 
    master_valve_state = VALVE_OPEN.copy()

    MOBILE_STATE_PATROL = "PATROL"; MOBILE_STATE_STOPPED = "STOPPED"; MOBILE_STATE_GO = "GO_TO_TARGET"
    MOBILE_STATE_TASK = "TASK"; MOBILE_STATE_WAIT = "WAIT_NORMAL"; MOBILE_STATE_RETURN = "RETURN_TO_PATROL"
    
    mobile_state = MOBILE_STATE_PATROL; mobile_task_timer = 0.0
    active_incident_code = None 

    def stop_mobile_base():
        qx, qy, qyaw = [float(v) for v in mobile_rig.get_base_dummy_q()]
        mobile_rig.set_base_dummy_q_with_hold(qx, qy, qyaw); mobile_rig.zero_all_velocities()

    def mobile_arm_home():
        q = mobile_rig.get_q().copy()
        for i in mobile_rig.other_idx: q[i] = float(mobile_home_q[i])
        mobile_rig.set_q(q); mobile_rig.snapshot_holds()

    # 💡 [조인트 헬퍼 함수] 목표 조인트 배열 생성 (스무스 보간용)
    def get_target_q_array(target_deg_dict, current_q):
        q_target = current_q.copy()
        dof_names = getattr(mobile_rig, 'dof_names', mobile_rig.articulation.dof_names if hasattr(mobile_rig, 'articulation') else [])
        for i, name in enumerate(dof_names):
            for t_name, t_val in target_deg_dict.items():
                if t_name in name:
                    q_target[i] = math.radians(t_val)
        return q_target

    def get_gripper_target_q(close, current_q):
        q_target = current_q.copy()
        dof_names = getattr(mobile_rig, 'dof_names', mobile_rig.articulation.dof_names if hasattr(mobile_rig, 'articulation') else [])
        for i, name in enumerate(dof_names):
            lower_name = name.lower()
            if 'finger' in lower_name or 'knuckle' in lower_name or 'gripper' in lower_name:
                q_target[i] = 0.8 if close else 0.0
        return q_target

    VALVE_APPROACH_Q_DEG = {
        'ur_arm_shoulder_pan_joint': -15.5406,
        'ur_arm_shoulder_lift_joint': -53.9696,
        'ur_arm_elbow_joint': 22.6059,
        'ur_arm_wrist_1_joint': -20.7782,
        'ur_arm_wrist_2_joint': 17.8363,
        'ur_arm_wrist_3_joint': 0.0011
    }
    
    VALVE_TURN_Q_DEG = VALVE_APPROACH_Q_DEG.copy()
    VALVE_TURN_Q_DEG['ur_arm_wrist_3_joint'] -= 90.0 # 6축 90도 회전

    drone_state = {
        'mode': 'patrol', 'idx': 0, 'is_hovering': False, 'stay_start_time': 0, 'scanned_in_current_slot': False,
        'current_command': None, 'override_step': 0, 'z_up_pos': None, 'reported_arrival': False
    }

    last_master_pub_time = 0.0
    last_gas_pub_time = 0.0
    
    # 💡 팔 시퀀스 상태 저장용 변수
    mobile_arm_state = "APPROACH"
    mobile_arm_start_q = None
    mobile_arm_target_q = None
    ag_valve_done = False

    print("\n🚀 [System Ready] Smooth Arm Interpolation & Auto-Normal Engine Active.")

    while simulation_app.is_running():
        world.step(render=not ARGS.headless)
        try: playing = world.is_playing()
        except Exception: playing = True
        if not playing: continue
        try: dt = float(world.get_physics_dt())
        except Exception: dt = 1.0/60.0

        current_time = time.time()
        
        gas_sim.update(current_time, dt)
        if current_time - last_gas_pub_time > 0.1:
            if global_ros_node:
                for z in ['A', 'B', 'C', 'D']:
                    msg = Float32(); msg.data = float(gas_sim.ppm[z])
                    global_ros_node.gas_ag_pubs[z].publish(msg)
                
                if active_incident_code:
                    gas_msg = String(); gas_msg.data = f"ppm 지수 : {gas_sim.ppm[active_incident_code]:.1f}"
                    global_ros_node.gas_pub.publish(gas_msg)
            last_gas_pub_time = current_time

        if current_time - last_master_pub_time > 1.0:
            if global_ros_node:
                global_ros_node.publish_master_state(
                    master_fsm_state, master_shutter_state, master_valve_state
                )
            last_master_pub_time = current_time
            
        # 💡 [핵심] 가스 0 도달 시 자동상황 종료 (Normal 이벤트 푸시)
        if mobile_state == MOBILE_STATE_WAIT and active_incident_code:
            if gas_sim.ppm[active_incident_code] <= 0.001 and gas_sim.ramp_active[active_incident_code]:
                print(f"\n[System] 🌿 {active_incident_code}구역 가스 농도 0% 도달. 상황을 자동으로 종료합니다.")
                eq.push(EvNormal())

        for ev in eq.pop_all():
            if isinstance(ev, EvGasStart):
                mobile_nav.clear_goal(); stop_mobile_base(); mobile_state = MOBILE_STATE_STOPPED
                cmd = ev.code
                active_incident_code = cmd
                print(f"[Mobile] 🛑 가스 경보 수신. 제자리 정지.")
                
                gas_sim.trigger_leak(cmd)
                if cmd in DRONE_COMMAND_TARGETS:
                    master_fsm_state[cmd] = 1 
                    print(f"\n🚨 [Drone] 긴급 명령 수신! '{cmd}' 지점으로 이동 시작!")
                    drone_state.update({'mode': 'override', 'current_command': cmd, 'override_step': 0, 'z_up_pos': None, 'reported_arrival': False})
            
            elif isinstance(ev, EvGasLocation):
                if mobile_state == MOBILE_STATE_STOPPED:
                    mobile_nav.set_goal(ev.x, ev.y, ev.yaw)
                    mobile_rig.snapshot_holds(); mobile_rig.zero_all_velocities(); mobile_state = MOBILE_STATE_GO
                    print(f"[Mobile] 🚀 가스 위치({ev.x}, {ev.y}) 수신. 출동!")
            
            elif isinstance(ev, EvIsolationDone):
                if mobile_state == MOBILE_STATE_TASK and active_incident_code == ev.code:
                    print(f"[Mobile] 🏁 액션그래프 밸브 잠금 완료 신호 수신!")
                    ag_valve_done = True

            elif isinstance(ev, EvNormal):
                if mobile_state in (MOBILE_STATE_WAIT, MOBILE_STATE_STOPPED):
                    qx, qy, _ = [float(v) for v in mobile_rig.get_base_dummy_q()]
                    return_anchor = min(patrol_pts, key=lambda p: (p[0] - qx)**2 + (p[1] - qy)**2)
                    mobile_nav.set_goal(return_anchor[0], return_anchor[1]); mobile_state = MOBILE_STATE_RETURN
                    print(f"[Mobile] 🔄 상황 종료. 순찰 지점으로 복귀 중...")
                
                if active_incident_code:
                    gas_sim.reset(active_incident_code)
                    master_fsm_state[active_incident_code] = 4
                    master_valve_state[active_incident_code] = VALVE_OPEN[active_incident_code]
                    for s in SHUTTER_MAPPING[active_incident_code]:
                        master_shutter_state[s] = False 
                active_incident_code = None

                print(f"\n✅ [Drone] 상황 종료. 순찰 재개!")
                drone_state.update({'mode': 'patrol', 'idx': 0, 'is_hovering': False, 'scanned_in_current_slot': False})

        # =========================================================
        # 🤖 모바일 로봇 동작 및 로봇 팔(Arm) 스무스 시퀀스
        # =========================================================
        if mobile_state == MOBILE_STATE_PATROL: mobile_patrol.step(dt)
        elif mobile_state == MOBILE_STATE_STOPPED: stop_mobile_base()
        elif mobile_state == MOBILE_STATE_GO:
            mobile_nav.step(dt)
            if mobile_nav.is_goal_reached():
                stop_mobile_base()
                if global_ros_node: global_ros_node.mobile_arrived_pub.publish(Empty())
                if active_incident_code:
                    for s in SHUTTER_MAPPING[active_incident_code]:
                        master_shutter_state[s] = True 
                    master_fsm_state[active_incident_code] = 2 
                
                # 시퀀스 변수 초기화
                mobile_task_timer = 0.0 
                mobile_arm_state = "APPROACH"
                mobile_arm_start_q = mobile_rig.get_q().copy()
                mobile_arm_target_q = get_target_q_array(VALVE_APPROACH_Q_DEG, mobile_arm_start_q)
                ag_valve_done = False
                mobile_state = MOBILE_STATE_TASK
                print(f"[Mobile] 🛠️ 도달 완료! 매니퓰레이터 밸브 조작 시퀀스(스무스 모션) 시작.")
        
        elif mobile_state == MOBILE_STATE_TASK:
            stop_mobile_base()
            mobile_task_timer += dt
            
            if mobile_arm_state in ["APPROACH", "GRIP_CLOSE", "TURN", "GRIP_OPEN", "HOME"]:
                duration = 2.0 if mobile_arm_state in ["APPROACH", "TURN", "HOME"] else 1.0
                progress = min(1.0, mobile_task_timer / duration)
                
                # 💡 [선형 보간 적용] 현재 진행도에 맞춰서 조인트 각도를 부드럽게 계산
                q_now = mobile_arm_start_q * (1.0 - progress) + mobile_arm_target_q * progress
                mobile_rig.set_q(q_now)
                mobile_rig.snapshot_holds()

                if mobile_task_timer >= duration:
                    if mobile_arm_state == "APPROACH":
                        mobile_arm_state = "GRIP_CLOSE"
                        mobile_arm_start_q = mobile_rig.get_q().copy()
                        mobile_arm_target_q = get_gripper_target_q(True, mobile_arm_start_q)
                        mobile_task_timer = 0.0
                        print(f"[Mobile] 로봇 팔 접근 완료. 그리퍼 닫기 진행.")
                        
                    elif mobile_arm_state == "GRIP_CLOSE":
                        mobile_arm_state = "TURN"
                        mobile_arm_start_q = mobile_rig.get_q().copy()
                        mobile_arm_target_q = get_target_q_array(VALVE_TURN_Q_DEG, mobile_arm_start_q)
                        mobile_task_timer = 0.0
                        print(f"[Mobile] 파지 완료. 6축 90도 회전 및 액션그래프 밸브 동기화.")
                        if active_incident_code:
                            master_valve_state[active_incident_code] = VALVE_CLOSE[active_incident_code]
                            
                    elif mobile_arm_state == "TURN":
                        mobile_arm_state = "GRIP_OPEN"
                        mobile_arm_start_q = mobile_rig.get_q().copy()
                        mobile_arm_target_q = get_gripper_target_q(False, mobile_arm_start_q)
                        mobile_task_timer = 0.0
                        print(f"[Mobile] 회전 완료. 그리퍼 열기 진행.")
                        
                    elif mobile_arm_state == "GRIP_OPEN":
                        mobile_arm_state = "HOME"
                        mobile_arm_start_q = mobile_rig.get_q().copy()
                        mobile_arm_target_q = mobile_rig.get_q().copy()
                        for idx in mobile_rig.other_idx: # Arm 조인트만 Home 각도로 복귀
                            mobile_arm_target_q[idx] = mobile_home_q[idx]
                        mobile_task_timer = 0.0
                        print(f"[Mobile] 그리퍼 개방. 팔을 원래 자세로 복귀.")
                        
                    elif mobile_arm_state == "HOME":
                        mobile_arm_state = "WAIT_AG_DONE"
                        mobile_task_timer = 0.0
                        print(f"[Mobile] 원위치 완료. 시스템(액션그래프) 최종 완료 신호 대기 중...")
                        
            elif mobile_arm_state == "WAIT_AG_DONE":
                if mobile_task_timer > 15.0 and not ag_valve_done:
                    print("[Mobile] ⚠️ 액션그래프 지연. 강제로 작업 완료 처리 진행.")
                    ag_valve_done = True
                    
                if ag_valve_done:
                    mobile_state = MOBILE_STATE_WAIT
                    if global_ros_node: 
                        global_ros_node.mobile_done_pub.publish(Empty())
                    if active_incident_code:
                        gas_sim.start_ramp_down(active_incident_code, current_time)
                        master_fsm_state[active_incident_code] = 3 
                    print("[Mobile] 📢 임무 완료 보고 완료. 가스 램프다운 진행 및 대기 모드 진입.")

        elif mobile_state == MOBILE_STATE_WAIT: stop_mobile_base()
        elif mobile_state == MOBILE_STATE_RETURN:
            mobile_nav.step(dt)
            if mobile_nav.is_goal_reached():
                mobile_nav.clear_goal(); mobile_patrol.reset_after_return(); mobile_rig.snapshot_holds(); mobile_rig.zero_all_velocities()
                mobile_state = MOBILE_STATE_PATROL
                print("[Mobile] 👮 순찰 모드(PATROL)로 복귀했습니다.")

        try:
            curr_pos, _ = drone.get_world_pose()
            if drone_state['mode'] == 'override':
                target_final = DRONE_COMMAND_TARGETS[drone_state['current_command']]
                if drone_state['override_step'] == 0:
                    if drone_state['z_up_pos'] is None: drone_state['z_up_pos'] = np.array([curr_pos[0], curr_pos[1], 7.0])
                    error = drone_state['z_up_pos'] - curr_pos; dist = np.linalg.norm(error)
                    if dist > 0.3: drone.set_linear_velocity((error / (dist + 1e-6)) * 4.0)
                    else: drone_state['override_step'] = 1; print(f"[Drone] ⬆️ 고도 확보. 가스 발생 지점으로 이동.")
                elif drone_state['override_step'] == 1:
                    error = target_final - curr_pos; dist = np.linalg.norm(error)
                    if dist > 0.3: drone.set_linear_velocity((error / (dist + 1e-6)) * 4.0)
                    else:
                        drone.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
                        drone_state.update({'override_step': 2})
                elif drone_state['override_step'] == 2:
                    if curr_pos[2] < 6.8: drone.set_linear_velocity(np.array([0.0, 0.0, (7.0 - curr_pos[2]) * 2.0]))
                    else: drone.set_linear_velocity(np.array([0.0, 0.0, 0.0]))

                    if not drone_state['reported_arrival'] and global_ros_node:
                        mx, my, myaw = DISPATCH_POINTS[drone_state['current_command']]
                        msg = String(); msg.data = f"{mx} {my} {myaw}"
                        global_ros_node.gas_loc_pub.publish(msg)
                        drone_state['reported_arrival'] = True
                        print(f"[Drone] 📍 모바일 로봇 호출: /main/gas_location '{msg.data}' 발행 완료")

            elif drone_state['mode'] == 'patrol':
                if drone_state['idx'] >= len(DRONE_SCAN_LOCATIONS):
                    drone_state.update({'idx': 0, 'is_hovering': False, 'stay_start_time': 0, 'scanned_in_current_slot': False})

                target = DRONE_SCAN_LOCATIONS[drone_state['idx']]
                error = target["pos"] - curr_pos; dist = np.linalg.norm(error)

                if not drone_state['is_hovering']:
                    if dist > 0.3: drone.set_linear_velocity((error / (dist + 1e-6)) * 4.0)
                    else:
                        if target["slot"] == "move": drone_state['idx'] += 1
                        else:
                            drone.set_linear_velocity(np.array([0, 0, 0]))
                            drone_state.update({'is_hovering': True, 'stay_start_time': time.time(), 'scanned_in_current_slot': False})
                else:
                    drone.set_linear_velocity(np.array([0, 0, 0]))
                    if not drone_state['scanned_in_current_slot'] and global_ros_node:
                        l_ok = scan_and_publish_barcode(rgb_annot_left, "Left", target["slot"], global_ros_node.barcode_pub)
                        r_ok = scan_and_publish_barcode(rgb_annot_right, "Right", target["slot"], global_ros_node.barcode_pub)
                        if l_ok or r_ok: drone_state['scanned_in_current_slot'] = True
                    if time.time() - drone_state['stay_start_time'] >= 3.0:
                        drone_state['is_hovering'] = False; drone_state['idx'] += 1

            drone.set_angular_velocity(np.array([0, 0, 0]))
            drone.set_joint_velocities(np.array([50.0, -50.0, 50.0, -50.0]))
        except Exception as e: pass

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\n[System] KeyboardInterrupt")
    except Exception as e:
        import traceback
        print("\n" + "="*50)
        print("🚨 치명적인 에러가 발생하여 종료되었습니다:")
        traceback.print_exc()
        print("="*50 + "\n")
    finally:
        try: simulation_app.close()
        except Exception: pass