#!/usr/bin/env python3
"""Novelty frontier explorer backed by the run_sim height-difference map.

This node keeps the map_sim mainline intact:
height-difference OccupancyGrid -> radar_known + novelty_map -> frontiers ->
Dijkstra -> local_goal/path -> GoalLock -> MPPI goal/reference path.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

try:
    from scipy.ndimage import distance_transform_edt
except Exception:  # pragma: no cover - fallback is for stripped runtime images.
    distance_transform_edt = None


UNKNOWN = 0
FREE = 1
OBSTACLE = 2
NEIGHBORS_8 = (
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
)
NEIGHBORS_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))


@dataclass
class MapMeta:
    frame_id: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    yaw: float


@dataclass
class RobotPose:
    stamp_sec: float
    x: float
    y: float
    yaw: float
    frame_id: str


class ExplorerParams:
    UNKNOWN = UNKNOWN
    FREE = FREE
    OBSTACLE = OBSTACLE

    def __init__(self) -> None:
        self.map_size = 20.0
        self.resolution = 0.1
        self.grid_n = 200
        self.radar_range = 5.0
        self.radar_rays = 180
        self.vision_range = 1.0
        self.vision_fov = 270.0
        self.vision_rays = 45
        self.novelty_init = 1.0
        self.novelty_decay = 0.5
        self.novelty_min = 0.0
        self.novelty_observed_threshold = 0.5
        self.candidate_min_dist = 0.3
        self.X = 30
        self.frontier_max_dist = 10.0
        self.frontier_visit_radius = 0.3
        self.boredom_decay = 0.95
        self.boredom_radius = 1.5
        self.w_boredom = 0.3
        self.w_momentum = 0.5
        self.d_step = 1.0
        self.d_min = 0.4
        self.d_max = 1.5
        self.safe_radius = 0.08
        self.trav_threshold = 0.22
        self.slope_threshold = 90.0
        self.roughness_threshold = 1e9
        self.w_obs = 1.0
        self.w_novelty = 2.0
        self.w_angle = 0.5
        self.w_path = 0.3
        self.arrived_threshold = 0.2
        self.stuck_steps = 3
        self.stuck_disp_threshold = 0.05
        self.path_jump_ratio = 0.5
        self.blacklist_ttl = 20
        self.blacklist_radius = 0.5


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    q = PoseStamped().pose.orientation
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def in_bounds(shape: Tuple[int, int], gi: int, gj: int) -> bool:
    return 0 <= gi < shape[0] and 0 <= gj < shape[1]


def shift_array(array: np.ndarray, delta_row: int, delta_col: int, fill_value):
    out = np.full(array.shape, fill_value, dtype=array.dtype)
    rows, cols = array.shape
    src_r0 = max(0, delta_row)
    src_r1 = min(rows, rows + delta_row)
    src_c0 = max(0, delta_col)
    src_c1 = min(cols, cols + delta_col)
    dst_r0 = max(0, -delta_row)
    dst_c0 = max(0, -delta_col)
    dst_r1 = dst_r0 + max(0, src_r1 - src_r0)
    dst_c1 = dst_c0 + max(0, src_c1 - src_c0)
    if dst_r1 > dst_r0 and dst_c1 > dst_c0:
        out[dst_r0:dst_r1, dst_c0:dst_c1] = array[src_r0:src_r1, src_c0:src_c1]
    return out


def shifted_cell(cell: Optional[Tuple[int, int]], delta_row: int, delta_col: int, shape):
    if cell is None:
        return None
    out = (cell[0] - delta_row, cell[1] - delta_col)
    return out if in_bounds(shape, out[0], out[1]) else None


def shifted_path(path: Optional[List[Tuple[int, int]]], delta_row: int, delta_col: int, shape):
    if not path:
        return None
    out = []
    for gi, gj in path:
        ngi, ngj = gi - delta_row, gj - delta_col
        if not in_bounds(shape, ngi, ngj):
            return None
        out.append((ngi, ngj))
    return out


def path_world_len(path: List[Tuple[int, int]], params: ExplorerParams) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
    return total * params.resolution


class GoalLock:
    def __init__(self, shape: Tuple[int, int], params: ExplorerParams) -> None:
        self.params = params
        self.current_goal: Optional[Tuple[int, int]] = None
        self.current_path: Optional[List[Tuple[int, int]]] = None
        self.last_path_len: Optional[float] = None
        self.blacklist: Dict[Tuple[int, int], int] = {}
        self.stuck_counter = 0
        self.last_robot_xy: Optional[Tuple[float, float]] = None
        self.frontier_visit_map = np.zeros(shape, dtype=bool)
        self.selected_frontier: Optional[Tuple[int, int]] = None
        self.fallback_mode = False
        self.boredom_map = np.zeros(shape, dtype=np.float32)
        self.last_explore_heading: Optional[float] = None
        self.explore_direction_ema: Optional[float] = None

    def set_goal(
        self,
        gi: int,
        gj: int,
        path: List[Tuple[int, int]],
        selected_frontier: Optional[Tuple[int, int]] = None,
        fallback_mode: bool = False,
        robot_xy: Optional[Tuple[float, float]] = None,
        robot_cell: Optional[Tuple[int, int]] = None,
    ) -> None:
        self.current_goal = (gi, gj)
        self.current_path = path
        self.last_path_len = path_world_len(path, self.params)
        self.selected_frontier = selected_frontier
        self.fallback_mode = fallback_mode
        self.stuck_counter = 0
        self.last_robot_xy = None
        if robot_cell is not None:
            ri, rj = robot_cell
            heading = math.atan2(gi - ri, gj - rj)
            self.last_explore_heading = heading
            if self.explore_direction_ema is None:
                self.explore_direction_ema = heading
            else:
                self.explore_direction_ema = wrap_angle(
                    self.explore_direction_ema + 0.3 * wrap_angle(heading - self.explore_direction_ema)
                )

    def clear_goal(self) -> None:
        self.current_goal = None
        self.current_path = None
        self.last_path_len = None
        self.selected_frontier = None
        self.fallback_mode = False

    def shift(self, delta_row: int, delta_col: int, shape: Tuple[int, int]) -> None:
        self.frontier_visit_map = shift_array(self.frontier_visit_map, delta_row, delta_col, False)
        self.boredom_map = shift_array(self.boredom_map, delta_row, delta_col, 0.0)
        self.current_goal = shifted_cell(self.current_goal, delta_row, delta_col, shape)
        self.current_path = shifted_path(self.current_path, delta_row, delta_col, shape)
        self.selected_frontier = shifted_cell(self.selected_frontier, delta_row, delta_col, shape)
        if self.current_goal is None or self.current_path is None:
            self.clear_goal()
        next_blacklist = {}
        for (gi, gj), ttl in self.blacklist.items():
            shifted = shifted_cell((gi, gj), delta_row, delta_col, shape)
            if shifted is not None:
                next_blacklist[shifted] = ttl
        self.blacklist = next_blacklist

    def add_blacklist(self, gi: int, gj: int) -> None:
        self.blacklist[(gi, gj)] = self.params.blacklist_ttl

    def tick_blacklist(self) -> None:
        for key in list(self.blacklist.keys()):
            self.blacklist[key] -= 1
            if self.blacklist[key] <= 0:
                del self.blacklist[key]

    def blacklist_set(self):
        return set(self.blacklist.keys())

    def is_blacklisted(self, gi: int, gj: int) -> bool:
        radius = self.params.blacklist_radius / self.params.resolution
        for bi, bj in self.blacklist:
            if math.hypot(bi - gi, bj - gj) <= radius:
                return True
        return False

    def mark_frontier_visited(self, fi: int, fj: int) -> None:
        rows, cols = self.frontier_visit_map.shape
        radius = int(self.params.frontier_visit_radius / self.params.resolution)
        r0, r1 = max(0, fi - radius), min(rows, fi + radius + 1)
        c0, c1 = max(0, fj - radius), min(cols, fj + radius + 1)
        yy, xx = np.mgrid[r0:r1, c0:c1]
        mask = (xx - fj) ** 2 + (yy - fi) ** 2 <= radius * radius
        self.frontier_visit_map[r0:r1, c0:c1] |= mask

    def visited_frontier_set(self):
        gi, gj = np.where(self.frontier_visit_map)
        return set(zip(gi.tolist(), gj.tolist()))

    def tick_boredom(self) -> None:
        self.boredom_map *= self.params.boredom_decay

    def add_boredom(self, gi: int, gj: int) -> None:
        rows, cols = self.boredom_map.shape
        radius = int(self.params.boredom_radius / self.params.resolution)
        r0, r1 = max(0, gi - radius), min(rows, gi + radius + 1)
        c0, c1 = max(0, gj - radius), min(cols, gj + radius + 1)
        yy, xx = np.mgrid[r0:r1, c0:c1]
        mask = (xx - gj) ** 2 + (yy - gi) ** 2 <= radius * radius
        self.boredom_map[r0:r1, c0:c1] += mask.astype(np.float32)

    def update_stuck(self, robot_xy: Tuple[float, float]) -> bool:
        if self.last_robot_xy is None:
            self.last_robot_xy = robot_xy
            self.stuck_counter = 0
            return False
        disp = math.hypot(robot_xy[0] - self.last_robot_xy[0], robot_xy[1] - self.last_robot_xy[1])
        if disp < self.params.stuck_disp_threshold:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        self.last_robot_xy = robot_xy
        return self.stuck_counter >= self.params.stuck_steps


def is_free(radar_known: np.ndarray, gi: int, gj: int, allow_unknown: bool) -> bool:
    if not in_bounds(radar_known.shape, gi, gj):
        return False
    if allow_unknown:
        return radar_known[gi, gj] != OBSTACLE
    return radar_known[gi, gj] == FREE


def dijkstra_from(
    radar_known: np.ndarray,
    start: Tuple[int, int],
    params: ExplorerParams,
    max_dist: Optional[float],
    allow_unknown: bool,
):
    if not is_free(radar_known, start[0], start[1], allow_unknown):
        return {}, {}
    max_cells = max_dist / params.resolution if max_dist else None
    open_set = [(0.0, start)]
    g_score = {start: 0.0}
    came_from = {}
    while open_set:
        dist, current = heapq.heappop(open_set)
        if dist > g_score.get(current, math.inf):
            continue
        if max_cells is not None and dist > max_cells:
            continue
        gi, gj = current
        for di, dj in NEIGHBORS_8:
            ni, nj = gi + di, gj + dj
            if not is_free(radar_known, ni, nj, allow_unknown):
                continue
            if di != 0 and dj != 0:
                if not is_free(radar_known, gi + di, gj, allow_unknown):
                    continue
                if not is_free(radar_known, gi, gj + dj, allow_unknown):
                    continue
            step = math.hypot(di, dj)
            tentative = dist + step
            nb = (ni, nj)
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_set, (tentative, nb))
    return g_score, came_from


def reconstruct_path(came_from, goal: Tuple[int, int]):
    path = [goal]
    while goal in came_from:
        goal = came_from[goal]
        path.append(goal)
    path.reverse()
    return path


def astar(radar_known: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], allow_unknown: bool):
    if not is_free(radar_known, start[0], start[1], allow_unknown):
        return None
    if not is_free(radar_known, goal[0], goal[1], allow_unknown):
        return None

    def h(cell):
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    open_set = [(h(start), 0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    while open_set:
        _, dist, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        if dist > g_score.get(current, math.inf):
            continue
        gi, gj = current
        for di, dj in NEIGHBORS_8:
            ni, nj = gi + di, gj + dj
            if not is_free(radar_known, ni, nj, allow_unknown):
                continue
            if di != 0 and dj != 0:
                if not is_free(radar_known, gi + di, gj, allow_unknown):
                    continue
                if not is_free(radar_known, gi, gj + dj, allow_unknown):
                    continue
            tentative = dist + math.hypot(di, dj)
            nb = (ni, nj)
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_set, (tentative + h(nb), tentative, nb))
    return None


def extract_frontiers(novelty_map: np.ndarray, radar_known: np.ndarray, params: ExplorerParams):
    observed = (novelty_map < params.novelty_observed_threshold) & (radar_known == FREE)
    unobserved = novelty_map >= params.novelty_observed_threshold
    adjacent_unobserved = np.zeros_like(observed, dtype=bool)
    adjacent_unobserved[:-1, :] |= unobserved[1:, :]
    adjacent_unobserved[1:, :] |= unobserved[:-1, :]
    adjacent_unobserved[:, :-1] |= unobserved[:, 1:]
    adjacent_unobserved[:, 1:] |= unobserved[:, :-1]
    gi, gj = np.where(observed & adjacent_unobserved)
    return list(zip(gi.tolist(), gj.tolist()))


def label_components(mask: np.ndarray):
    labels = np.zeros(mask.shape, dtype=np.int32)
    sizes = [0]
    sums = [0.0]
    current = 0
    rows, cols = mask.shape
    for gi in range(rows):
        for gj in range(cols):
            if not mask[gi, gj] or labels[gi, gj] != 0:
                continue
            current += 1
            q = deque([(gi, gj)])
            labels[gi, gj] = current
            size = 0
            while q:
                ci, cj = q.popleft()
                size += 1
                for di, dj in NEIGHBORS_4:
                    ni, nj = ci + di, cj + dj
                    if 0 <= ni < rows and 0 <= nj < cols and mask[ni, nj] and labels[ni, nj] == 0:
                        labels[ni, nj] = current
                        q.append((ni, nj))
            sizes.append(size)
            sums.append(0.0)
    return labels, np.asarray(sizes, dtype=np.float32), np.asarray(sums, dtype=np.float32)


def compute_frontier_weights(
    novelty_map: np.ndarray,
    radar_known: np.ndarray,
    frontier_cells: List[Tuple[int, int]],
    params: ExplorerParams,
):
    if not frontier_cells:
        return {}
    unobserved_free = (novelty_map >= params.novelty_observed_threshold) & (radar_known == FREE)
    labels, comp_sizes, _ = label_components(unobserved_free)
    if len(comp_sizes) <= 1:
        return {cell: 0.0 for cell in frontier_cells}
    novelty_sums = np.bincount(labels.ravel(), weights=novelty_map.ravel(), minlength=len(comp_sizes))
    with np.errstate(divide="ignore", invalid="ignore"):
        avg_novelty = np.where(comp_sizes > 0, novelty_sums / np.maximum(comp_sizes, 1.0), 0.0)
    comp_weights = comp_sizes * avg_novelty
    rows, cols = novelty_map.shape
    weights = {}
    for gi, gj in frontier_cells:
        best = 0.0
        for di, dj in NEIGHBORS_8:
            ni, nj = gi + di, gj + dj
            if 0 <= ni < rows and 0 <= nj < cols:
                label_id = labels[ni, nj]
                if label_id > 0:
                    best = max(best, float(comp_weights[label_id]))
        weights[(gi, gj)] = best
    return weights


def select_frontiers(
    frontier_cells: List[Tuple[int, int]],
    robot_cell: Tuple[int, int],
    params: ExplorerParams,
    visited_set,
    frontier_weights,
    max_dist: float,
    g_score,
    explore_direction: Optional[float] = None,
):
    scored = []
    for gi, gj in frontier_cells:
        if visited_set and (gi, gj) in visited_set:
            continue
        if (gi, gj) not in g_score:
            continue
        dist_m = g_score[(gi, gj)] * params.resolution
        if dist_m > max_dist:
            continue
        weight = max(frontier_weights.get((gi, gj), 1.0), 0.1)
        score = weight / (dist_m + 1.0)
        if explore_direction is not None:
            dx = gi - robot_cell[0]
            dy = gj - robot_cell[1]
            if dx != 0 or dy != 0:
                dir_to_frontier = math.atan2(dy, dx)
                dir_diff = abs(wrap_angle(dir_to_frontier - explore_direction))
                momentum_bonus = 1.0 + 0.5 * (math.cos(dir_diff) + 1.0)
                score *= momentum_bonus
        scored.append((-(score), dist_m, gi, gj))
    scored.sort()
    selected = []
    min_cells = params.candidate_min_dist / params.resolution
    for _, _, gi, gj in scored:
        if all(math.hypot(gi - si, gj - sj) >= min_cells for si, sj in selected):
            selected.append((gi, gj))
            if len(selected) >= params.X:
                break
    return selected


def cut_local_goal_and_path(path: List[Tuple[int, int]], params: ExplorerParams):
    if len(path) < 2:
        return None, None
    cum = [0.0]
    segs = []
    for i in range(len(path) - 1):
        seg = math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        segs.append(seg)
        cum.append(cum[-1] + seg)
    total = cum[-1]
    d_min = params.d_min / params.resolution
    d_step = params.d_step / params.resolution
    d_max = params.d_max / params.resolution
    if total < d_min:
        return None, None
    target = min(d_step, total, d_max)
    for i, seg in enumerate(segs):
        if cum[i + 1] >= target:
            frac = (target - cum[i]) / seg if seg > 1e-9 else 0.0
            gi = path[i][0] + frac * (path[i + 1][0] - path[i][0])
            gj = path[i][1] + frac * (path[i + 1][1] - path[i][1])
            local_goal = (int(round(gi)), int(round(gj)))
            return local_goal, path[: i + 1] + [local_goal]
    return path[-1], path


def obstacle_distance_cells(obstacle: np.ndarray):
    if not np.any(obstacle):
        return np.full(obstacle.shape, np.inf, dtype=np.float32)
    if distance_transform_edt is not None:
        return distance_transform_edt(~obstacle).astype(np.float32)
    dist = np.full(obstacle.shape, np.inf, dtype=np.float32)
    open_set = []
    for gi, gj in np.argwhere(obstacle):
        dist[gi, gj] = 0.0
        heapq.heappush(open_set, (0.0, int(gi), int(gj)))
    while open_set:
        cur, gi, gj = heapq.heappop(open_set)
        if cur > dist[gi, gj]:
            continue
        for di, dj in NEIGHBORS_8:
            ni, nj = gi + di, gj + dj
            if in_bounds(obstacle.shape, ni, nj):
                nd = cur + math.hypot(di, dj)
                if nd < dist[ni, nj]:
                    dist[ni, nj] = nd
                    heapq.heappush(open_set, (nd, ni, nj))
    return dist


def angle_cost(local_goal_xy, robot_xy, robot_heading):
    diff = wrap_angle(math.atan2(local_goal_xy[1] - robot_xy[1], local_goal_xy[0] - robot_xy[0]) - robot_heading)
    return (1.0 - math.cos(diff)) * 0.5


class NoveltyExplorer(Node):
    def __init__(self) -> None:
        super().__init__("novelty_explorer")
        self.declare_parameters(
            "",
            [
                ("traversability_map_topic", "/traversability_map"),
                ("odom_topic", "/odom"),
                ("world_pose_topic", "/pose"),
                ("goal_topic", "/goal_pose"),
                ("reference_path_topic", "/mppi/reference_path"),
                ("radar_known_debug_topic", "/novelty_explorer/radar_known"),
                ("novelty_debug_topic", "/novelty_explorer/novelty_map"),
                ("planning_rate", 2.0),
                ("pose_timeout_sec", 1.0),
                ("map_timeout_sec", 2.0),
                ("goal_republish_sec", 1.0),
                ("occupancy_obstacle_threshold", 75),
                ("radar_range", 5.0),
                ("radar_rays", 180),
                ("vision_range", 1.0),
                ("vision_fov", 270.0),
                ("vision_rays", 45),
                ("novelty_decay", 0.5),
                ("novelty_observed_threshold", 0.5),
                ("candidate_min_dist", 0.3),
                ("frontier_count", 30),
                ("frontier_max_dist", 10.0),
                ("d_step", 1.0),
                ("d_min", 0.4),
                ("d_max", 1.5),
                ("safe_radius", 0.08),
                ("trav_threshold", 0.22),
                ("arrived_threshold", 0.2),
                ("stuck_steps", 3),
                ("stuck_disp_threshold", 0.05),
                ("enable_unknown_fallback", True),
                ("enable_last_resort_no_los", True),
            ],
        )

        self.params = ExplorerParams()
        self.params.radar_range = float(self.get_parameter("radar_range").value)
        self.params.radar_rays = int(self.get_parameter("radar_rays").value)
        self.params.vision_range = float(self.get_parameter("vision_range").value)
        self.params.vision_fov = float(self.get_parameter("vision_fov").value)
        self.params.vision_rays = int(self.get_parameter("vision_rays").value)
        self.params.novelty_decay = float(self.get_parameter("novelty_decay").value)
        self.params.novelty_observed_threshold = float(self.get_parameter("novelty_observed_threshold").value)
        self.params.candidate_min_dist = float(self.get_parameter("candidate_min_dist").value)
        self.params.X = int(self.get_parameter("frontier_count").value)
        self.params.frontier_max_dist = float(self.get_parameter("frontier_max_dist").value)
        self.params.d_step = float(self.get_parameter("d_step").value)
        self.params.d_min = float(self.get_parameter("d_min").value)
        self.params.d_max = float(self.get_parameter("d_max").value)
        self.params.safe_radius = float(self.get_parameter("safe_radius").value)
        self.params.trav_threshold = float(self.get_parameter("trav_threshold").value)
        self.params.arrived_threshold = float(self.get_parameter("arrived_threshold").value)
        self.params.stuck_steps = int(self.get_parameter("stuck_steps").value)
        self.params.stuck_disp_threshold = float(self.get_parameter("stuck_disp_threshold").value)

        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.map_timeout_sec = float(self.get_parameter("map_timeout_sec").value)
        self.goal_republish_sec = float(self.get_parameter("goal_republish_sec").value)
        self.occupancy_obstacle_threshold = int(self.get_parameter("occupancy_obstacle_threshold").value)
        self.enable_unknown_fallback = bool(self.get_parameter("enable_unknown_fallback").value)
        self.enable_last_resort_no_los = bool(self.get_parameter("enable_last_resort_no_los").value)

        latch_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("traversability_map_topic").value),
            self.on_map,
            latch_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.on_odom,
            20,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("world_pose_topic").value),
            self.on_world_pose,
            20,
        )
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.path_pub = self.create_publisher(Path, str(self.get_parameter("reference_path_topic").value), 10)
        self.radar_debug_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("radar_known_debug_topic").value),
            latch_qos,
        )
        self.novelty_debug_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("novelty_debug_topic").value),
            latch_qos,
        )

        self.meta: Optional[MapMeta] = None
        self.latest_map_stamp = 0.0
        self.obstacle = None
        self.observed_mask = None
        self.traversability = None
        self.obs_dist = None
        self.radar_known = None
        self.novelty_map = None
        self.goal_lock: Optional[GoalLock] = None
        self.latest_pose: Optional[RobotPose] = None
        self.last_goal_publish_sec = 0.0
        self.last_goal_cell: Optional[Tuple[int, int]] = None
        self.no_frontier_count = 0

        timer_period = 1.0 / max(float(self.get_parameter("planning_rate").value), 0.1)
        self.timer = self.create_timer(timer_period, self.on_timer)
        self.get_logger().info(
            "novelty_explorer ready: height-difference map -> radar_known -> /goal_pose + /mppi/reference_path"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_world_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = RobotPose(
            self.now_sec(),
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            yaw_from_quaternion(msg.pose.orientation),
            msg.header.frame_id.strip() or "world",
        )

    def on_odom(self, msg: Odometry) -> None:
        if self.latest_pose is not None and self.now_sec() - self.latest_pose.stamp_sec <= self.pose_timeout_sec:
            return
        pose = msg.pose.pose
        self.latest_pose = RobotPose(
            self.now_sec(),
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
            msg.header.frame_id.strip() or "world",
        )

    def on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            self.get_logger().warning("Ignoring malformed traversability map.")
            return
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            self.get_logger().warning("Ignoring traversability map with invalid resolution.")
            return
        frame_id = msg.header.frame_id.strip() or "world"
        yaw = yaw_from_quaternion(msg.info.origin.orientation)
        next_meta = MapMeta(
            frame_id,
            width,
            height,
            resolution,
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            yaw,
        )
        self.align_state(next_meta)
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        known = data >= 0
        clipped = np.clip(data, 0, 100).astype(np.float32)
        self.observed_mask = known
        self.obstacle = known & (data >= self.occupancy_obstacle_threshold)
        self.traversability = np.where(known, 1.0 - clipped / 100.0, 0.0).astype(np.float32)
        self.obs_dist = obstacle_distance_cells(self.obstacle)
        self.latest_map_stamp = self.now_sec()

    def align_state(self, next_meta: MapMeta) -> None:
        shape = (next_meta.height, next_meta.width)
        self.params.grid_n = next_meta.height
        self.params.resolution = next_meta.resolution
        self.params.map_size = max(next_meta.height, next_meta.width) * next_meta.resolution
        if self.meta is None or self.radar_known is None or self.novelty_map is None or self.goal_lock is None:
            self.meta = next_meta
            self.radar_known = np.full(shape, UNKNOWN, dtype=np.uint8)
            self.novelty_map = np.full(shape, self.params.novelty_init, dtype=np.float32)
            self.goal_lock = GoalLock(shape, self.params)
            return

        same_grid = (
            self.meta.width == next_meta.width
            and self.meta.height == next_meta.height
            and abs(self.meta.resolution - next_meta.resolution) < 1e-6
            and abs(wrap_angle(self.meta.yaw - next_meta.yaw)) < 1e-3
            and self.meta.frame_id == next_meta.frame_id
        )
        if not same_grid:
            self.meta = next_meta
            self.radar_known = np.full(shape, UNKNOWN, dtype=np.uint8)
            self.novelty_map = np.full(shape, self.params.novelty_init, dtype=np.float32)
            self.goal_lock = GoalLock(shape, self.params)
            return

        dx = next_meta.origin_x - self.meta.origin_x
        dy = next_meta.origin_y - self.meta.origin_y
        c = math.cos(self.meta.yaw)
        s = math.sin(self.meta.yaw)
        local_dx = c * dx + s * dy
        local_dy = -s * dx + c * dy
        delta_col = int(round(local_dx / next_meta.resolution))
        delta_row = int(round(local_dy / next_meta.resolution))
        if delta_row != 0 or delta_col != 0:
            self.radar_known = shift_array(self.radar_known, delta_row, delta_col, UNKNOWN)
            self.novelty_map = shift_array(self.novelty_map, delta_row, delta_col, self.params.novelty_init)
            self.goal_lock.shift(delta_row, delta_col, shape)
        self.meta = next_meta

    def world_to_grid_float(self, x: float, y: float):
        assert self.meta is not None
        dx = x - self.meta.origin_x
        dy = y - self.meta.origin_y
        c = math.cos(self.meta.yaw)
        s = math.sin(self.meta.yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return local_y / self.meta.resolution, local_x / self.meta.resolution

    def world_to_grid(self, x: float, y: float):
        gi_f, gj_f = self.world_to_grid_float(x, y)
        return int(math.floor(gi_f)), int(math.floor(gj_f))

    def grid_to_world(self, gi: int, gj: int):
        assert self.meta is not None
        local_x = (gj + 0.5) * self.meta.resolution
        local_y = (gi + 0.5) * self.meta.resolution
        c = math.cos(self.meta.yaw)
        s = math.sin(self.meta.yaw)
        x = self.meta.origin_x + c * local_x - s * local_y
        y = self.meta.origin_y + s * local_x + c * local_y
        return x, y

    def ray_cells(self, x: float, y: float, angle: float, max_range: float):
        assert self.meta is not None
        step = self.meta.resolution * 0.5
        n_steps = int(max_range / step)
        dx = math.cos(angle)
        dy = math.sin(angle)
        for i in range(1, n_steps + 1):
            yield self.world_to_grid(x + dx * step * i, y + dy * step * i)

    def update_radar_from_height_map(self, pose: RobotPose) -> None:
        assert self.radar_known is not None and self.observed_mask is not None and self.obstacle is not None
        for i in range(max(self.params.radar_rays, 1)):
            angle = 2.0 * math.pi * i / max(self.params.radar_rays, 1)
            for gi, gj in self.ray_cells(pose.x, pose.y, angle, self.params.radar_range):
                if not in_bounds(self.radar_known.shape, gi, gj):
                    break
                if self.obstacle[gi, gj]:
                    self.radar_known[gi, gj] = OBSTACLE
                    break
                self.radar_known[gi, gj] = FREE

    def update_novelty_from_virtual_vision(self, pose: RobotPose) -> None:
        assert self.novelty_map is not None and self.obstacle is not None and self.observed_mask is not None
        half_fov = math.radians(self.params.vision_fov * 0.5)
        for i in range(max(self.params.vision_rays, 1)):
            rel = -half_fov + 2.0 * half_fov * i / max(self.params.vision_rays - 1, 1)
            for gi, gj in self.ray_cells(pose.x, pose.y, pose.yaw + rel, self.params.vision_range):
                if not in_bounds(self.novelty_map.shape, gi, gj):
                    break
                if self.obstacle[gi, gj]:
                    break
                self.novelty_map[gi, gj] = max(
                    self.params.novelty_min,
                    self.novelty_map[gi, gj] - self.params.novelty_decay,
                )

    def simulate_novelty_gain(self, x: float, y: float, heading: float) -> float:
        assert self.novelty_map is not None and self.obstacle is not None and self.observed_mask is not None
        half_fov = math.radians(self.params.vision_fov * 0.5)
        gain = 0.0
        for i in range(max(self.params.vision_rays, 1)):
            rel = -half_fov + 2.0 * half_fov * i / max(self.params.vision_rays - 1, 1)
            for gi, gj in self.ray_cells(x, y, heading + rel, self.params.vision_range):
                if not in_bounds(self.novelty_map.shape, gi, gj):
                    break
                if self.obstacle[gi, gj]:
                    break
                gain += min(float(self.novelty_map[gi, gj]), self.params.novelty_decay)
        return gain

    def is_radar_visible(self, pose: RobotPose, target: Tuple[int, int]) -> bool:
        tx, ty = self.grid_to_world(target[0], target[1])
        dx = tx - pose.x
        dy = ty - pose.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return True
        angle = math.atan2(dy, dx)
        assert self.obstacle is not None and self.observed_mask is not None
        for gi, gj in self.ray_cells(pose.x, pose.y, angle, dist):
            if not in_bounds(self.obstacle.shape, gi, gj):
                return False
            if self.obstacle[gi, gj]:
                return False
        return True

    def collect_goal_candidates(
        self,
        pose: RobotPose,
        candidates: List[Tuple[int, int]],
        came_from,
        frontier_weights,
        max_fw: float,
        allow_unknown: bool,
        check_los: bool,
    ):
        results = []
        for frontier in candidates:
            if check_los and allow_unknown and not self.is_radar_visible(pose, frontier):
                continue
            full_path = reconstruct_path(came_from, frontier)
            local_goal, path_to_lg = cut_local_goal_and_path(full_path, self.params)
            if local_goal is None or path_to_lg is None:
                continue
            if not self.is_safe_local_goal(local_goal[0], local_goal[1], allow_unknown):
                continue
            if not self.path_safety_check(path_to_lg, allow_unknown):
                continue
            local_goal_xy = self.grid_to_world(local_goal[0], local_goal[1])
            cost = self.compute_cost(
                local_goal_xy,
                path_to_lg,
                (pose.x, pose.y),
                pose.yaw,
                frontier_weights.get(frontier, 0.0),
                max_fw,
            )
            results.append((cost, local_goal, path_to_lg, frontier))
        return results

    def is_safe_local_goal(self, gi: int, gj: int, allow_unknown: bool) -> bool:
        assert self.radar_known is not None and self.obstacle is not None and self.obs_dist is not None
        assert self.traversability is not None and self.goal_lock is not None
        if not in_bounds(self.radar_known.shape, gi, gj):
            return False
        state = self.radar_known[gi, gj]
        if allow_unknown:
            if state == OBSTACLE or self.obstacle[gi, gj]:
                return False
        elif state != FREE:
            return False
        if self.goal_lock.is_blacklisted(gi, gj):
            return False
        if allow_unknown:
            return True
        if self.obs_dist[gi, gj] < self.params.safe_radius / self.params.resolution:
            return False
        if self.traversability[gi, gj] < self.params.trav_threshold:
            return False
        return True

    def path_safety_check(self, path: List[Tuple[int, int]], allow_unknown: bool) -> bool:
        assert self.radar_known is not None and self.obs_dist is not None and self.traversability is not None
        for gi, gj in path:
            if not in_bounds(self.radar_known.shape, gi, gj):
                return False
            state = self.radar_known[gi, gj]
            if state == OBSTACLE:
                return False
            if not allow_unknown:
                if state != FREE:
                    return False
                if self.obs_dist[gi, gj] < self.params.safe_radius / self.params.resolution:
                    return False
                if self.traversability[gi, gj] < self.params.trav_threshold:
                    return False
        return True

    def compute_cost(
        self,
        local_goal_xy,
        path_to_lg: List[Tuple[int, int]],
        robot_xy,
        robot_heading: float,
        frontier_weight: float,
        max_frontier_weight: float,
    ) -> float:
        assert self.obs_dist is not None and self.traversability is not None and self.goal_lock is not None
        gi, gj = self.world_to_grid(local_goal_xy[0], local_goal_xy[1])
        obs_d_m = float(self.obs_dist[gi, gj]) * self.params.resolution
        c_obs_obs = math.exp(-obs_d_m / 0.5)
        c_obs_trav = 1.0 - float(self.traversability[gi, gj])
        c_obs = 0.5 * c_obs_obs + 0.5 * c_obs_trav
        face_heading = math.atan2(local_goal_xy[1] - robot_xy[1], local_goal_xy[0] - robot_xy[0])
        gain = self.simulate_novelty_gain(local_goal_xy[0], local_goal_xy[1], face_heading)
        ray_step = self.meta.resolution * 0.5
        max_gain = self.params.vision_rays * (self.params.vision_range / ray_step) * self.params.novelty_decay
        c_novelty_local = 1.0 - min(gain / max_gain, 1.0) if max_gain > 0.0 else 1.0
        c_novelty_area = 1.0 - min(frontier_weight / max_frontier_weight, 1.0) if max_frontier_weight > 0.0 else 1.0
        c_novelty = 0.5 * c_novelty_local + 0.5 * c_novelty_area
        c_angle = angle_cost(local_goal_xy, robot_xy, robot_heading)
        c_path = min(path_world_len(path_to_lg, self.params) / self.params.radar_range, 1.0)
        c_boredom = min(float(self.goal_lock.boredom_map[gi, gj]) / 3.0, 1.0)
        c_momentum = 0.0
        if self.goal_lock.explore_direction_ema is not None:
            robot_cell = self.world_to_grid(robot_xy[0], robot_xy[1])
            dir_to_goal = math.atan2(gi - robot_cell[0], gj - robot_cell[1])
            diff = abs(wrap_angle(dir_to_goal - self.goal_lock.explore_direction_ema))
            c_momentum = (1.0 - math.cos(diff)) * 0.5
        return (
            self.params.w_obs * c_obs
            + self.params.w_novelty * c_novelty
            + self.params.w_angle * c_angle
            + self.params.w_path * c_path
            + self.params.w_boredom * c_boredom
            + self.params.w_momentum * c_momentum
        )

    def should_release_goal(self, pose: RobotPose, stuck: bool):
        assert self.goal_lock is not None and self.radar_known is not None
        if self.goal_lock.current_goal is None:
            return True, "no_goal"
        goal = self.goal_lock.current_goal
        if not in_bounds(self.radar_known.shape, goal[0], goal[1]):
            return True, "oob"
        if self.radar_known[goal[0], goal[1]] == OBSTACLE:
            return True, "became_obstacle"
        if not self.goal_lock.fallback_mode and self.radar_known[goal[0], goal[1]] == UNKNOWN:
            return True, "became_unknown"
        start = self.world_to_grid(pose.x, pose.y)
        new_path = astar(self.radar_known, start, goal, self.goal_lock.fallback_mode)
        if new_path is None:
            return True, "unreachable"
        if not self.path_safety_check(new_path, self.goal_lock.fallback_mode):
            return True, "path_unsafe"
        new_len = path_world_len(new_path, self.params)
        if self.goal_lock.last_path_len and self.goal_lock.last_path_len > 0.05:
            if abs(new_len - self.goal_lock.last_path_len) / self.goal_lock.last_path_len > self.params.path_jump_ratio:
                return True, "path_jump"
        if stuck:
            return True, "stuck"
        self.goal_lock.current_path = new_path
        self.goal_lock.last_path_len = new_len
        return False, "ok"

    def choose_goal(self, pose: RobotPose):
        assert self.radar_known is not None and self.novelty_map is not None and self.goal_lock is not None
        robot_cell = self.world_to_grid(pose.x, pose.y)
        if not in_bounds(self.radar_known.shape, robot_cell[0], robot_cell[1]):
            self.get_logger().warning("Robot pose is outside the traversability map; exploration waits.")
            return False
        frontiers = extract_frontiers(self.novelty_map, self.radar_known, self.params)
        if not frontiers:
            self.no_frontier_count += 1
            return False
        self.no_frontier_count = 0
        frontier_weights = compute_frontier_weights(self.novelty_map, self.radar_known, frontiers, self.params)
        max_fw = max(frontier_weights.values()) if frontier_weights else 1.0
        visited = self.goal_lock.visited_frontier_set()
        levels = [(False, 1.0)]
        if self.enable_unknown_fallback:
            levels.extend([(True, 2.0), (True, 4.0), (True, 8.0)])

        for allow_unknown, dist_mult in levels:
            max_dist = min(self.params.frontier_max_dist * dist_mult, self.params.map_size)
            g_score, came_from = dijkstra_from(self.radar_known, robot_cell, self.params, max_dist, allow_unknown)
            candidates = select_frontiers(frontiers, robot_cell, self.params, visited, frontier_weights, max_dist, g_score, self.goal_lock.explore_direction_ema if self.goal_lock else None)
            results = self.collect_goal_candidates(
                pose,
                candidates,
                came_from,
                frontier_weights,
                max_fw,
                allow_unknown,
                check_los=True,
            )
            if results:
                results.sort(key=lambda item: item[0])
                _, local_goal, path_to_lg, frontier = results[0]
                self.goal_lock.set_goal(
                    local_goal[0],
                    local_goal[1],
                    path_to_lg,
                    selected_frontier=frontier,
                    fallback_mode=allow_unknown,
                    robot_xy=(pose.x, pose.y),
                    robot_cell=robot_cell,
                )
                self.get_logger().info(
                    f"Exploration goal set: cell=({local_goal[0]},{local_goal[1]}) "
                    f"frontier=({frontier[0]},{frontier[1]}) "
                    f"fallback={allow_unknown} frontiers={len(frontiers)}"
                )
                self.publish_goal_and_path(force=True)
                return True

        if self.enable_unknown_fallback and self.enable_last_resort_no_los:
            g_score, came_from = dijkstra_from(
                self.radar_known,
                robot_cell,
                self.params,
                self.params.map_size,
                allow_unknown=True,
            )
            candidates = select_frontiers(
                frontiers,
                robot_cell,
                self.params,
                visited,
                frontier_weights,
                self.params.map_size,
                g_score,
                self.goal_lock.explore_direction_ema if self.goal_lock else None,
            )
            results = self.collect_goal_candidates(
                pose,
                candidates,
                came_from,
                frontier_weights,
                max_fw,
                allow_unknown=True,
                check_los=False,
            )
            if results:
                results.sort(key=lambda item: item[0])
                _, local_goal, path_to_lg, frontier = results[0]
                self.goal_lock.set_goal(
                    local_goal[0],
                    local_goal[1],
                    path_to_lg,
                    selected_frontier=frontier,
                    fallback_mode=True,
                    robot_xy=(pose.x, pose.y),
                    robot_cell=robot_cell,
                )
                self.get_logger().info(
                    f"Exploration goal set: cell=({local_goal[0]},{local_goal[1]}) "
                    f"frontier=({frontier[0]},{frontier[1]}) "
                    f"fallback=True last_resort=True frontiers={len(frontiers)}"
                )
                self.publish_goal_and_path(force=True)
                return True

        self.get_logger().warning(
            f"Exploration planning failed: frontiers={len(frontiers)} "
            f"visited={len(visited)} blacklist={len(self.goal_lock.blacklist)}"
        )
        return False

    def publish_goal_and_path(self, force: bool = False) -> None:
        assert self.meta is not None and self.goal_lock is not None
        if self.goal_lock.current_goal is None or not self.goal_lock.current_path:
            return
        now = self.now_sec()
        if not force and now - self.last_goal_publish_sec < self.goal_republish_sec:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.meta.frame_id
        path_points = []
        for gi, gj in self.goal_lock.current_path:
            x, y = self.grid_to_world(gi, gj)
            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.orientation.w = 1.0
            path_msg.poses.append(pose_msg)
            path_points.append((x, y))

        goal_x, goal_y = self.grid_to_world(*self.goal_lock.current_goal)
        if len(path_points) >= 2:
            prev_x, prev_y = path_points[-2]
            goal_yaw = math.atan2(goal_y - prev_y, goal_x - prev_x)
        else:
            goal_yaw = 0.0
        goal_msg = PoseStamped()
        goal_msg.header = path_msg.header
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation = yaw_to_quaternion(goal_yaw)

        self.path_pub.publish(path_msg)
        self.goal_pub.publish(goal_msg)
        self.last_goal_publish_sec = now
        self.last_goal_cell = self.goal_lock.current_goal

    def publish_debug_maps(self) -> None:
        if self.meta is None or self.radar_known is None or self.novelty_map is None:
            return
        stamp = self.get_clock().now().to_msg()
        radar = self.make_grid_msg(stamp)
        radar.data = np.where(
            self.radar_known == UNKNOWN,
            -1,
            np.where(self.radar_known == OBSTACLE, 100, 0),
        ).astype(np.int8).ravel(order="C").tolist()
        self.radar_debug_pub.publish(radar)

        novelty = self.make_grid_msg(stamp)
        novelty_data = np.rint(np.clip(1.0 - self.novelty_map, 0.0, 1.0) * 100.0).astype(np.int8)
        novelty.data = novelty_data.ravel(order="C").tolist()
        self.novelty_debug_pub.publish(novelty)

    def make_grid_msg(self, stamp):
        assert self.meta is not None
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.meta.frame_id
        msg.info.resolution = self.meta.resolution
        msg.info.width = self.meta.width
        msg.info.height = self.meta.height
        msg.info.origin.position.x = self.meta.origin_x
        msg.info.origin.position.y = self.meta.origin_y
        msg.info.origin.orientation = yaw_to_quaternion(self.meta.yaw)
        return msg

    def on_timer(self) -> None:
        if self.meta is None or self.latest_pose is None or self.radar_known is None:
            return
        now = self.now_sec()
        if now - self.latest_map_stamp > self.map_timeout_sec:
            self.get_logger().warning("Exploration waits for a fresh traversability map.")
            return
        if now - self.latest_pose.stamp_sec > self.pose_timeout_sec:
            self.get_logger().warning("Exploration waits for a fresh robot pose.")
            return

        pose = self.latest_pose
        robot_cell = self.world_to_grid(pose.x, pose.y)
        if not in_bounds(self.radar_known.shape, robot_cell[0], robot_cell[1]):
            return
        self.update_radar_from_height_map(pose)
        self.update_novelty_from_virtual_vision(pose)
        assert self.goal_lock is not None
        self.goal_lock.tick_boredom()
        self.goal_lock.add_boredom(robot_cell[0], robot_cell[1])
        stuck = self.goal_lock.update_stuck((pose.x, pose.y))

        if self.goal_lock.current_goal is not None:
            release, reason = self.should_release_goal(pose, stuck)
            if release:
                if reason != "no_goal":
                    self.get_logger().info(f"Releasing exploration goal: {reason}")
                    if self.goal_lock.current_goal is not None:
                        self.goal_lock.add_blacklist(*self.goal_lock.current_goal)
                self.goal_lock.clear_goal()

        if self.goal_lock.current_goal is None:
            self.choose_goal(pose)
        else:
            goal_xy = self.grid_to_world(*self.goal_lock.current_goal)
            if math.hypot(pose.x - goal_xy[0], pose.y - goal_xy[1]) < self.params.arrived_threshold:
                if self.goal_lock.selected_frontier is not None:
                    self.goal_lock.mark_frontier_visited(*self.goal_lock.selected_frontier)
                self.goal_lock.clear_goal()
            else:
                self.publish_goal_and_path()

        self.goal_lock.tick_blacklist()
        self.publish_debug_maps()


def main() -> None:
    rclpy.init()
    node = NoveltyExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
