"""Explore: drive the robot through the house in TestMode and map it, no cleaning.

The robot's wheel odometry is very accurate in TestMode (a drive test on a D7
came back within 3 mm), but it has no SLAM there, so each scan is also matched
against the map built so far to stop slow drift. Then the classic loop:
scan, add it to the map, find the nearest edge between known floor and
unknown space (a frontier), plan a path there that keeps the robot clear of
walls, drive a short piece of it, repeat. When nothing is left to see, or on
low battery, a time limit or a stop request, it drives back and reverses onto
the dock.

All motion goes through Driver, which checks bumpers, lifted wheels and cliff
sensors before and during every move and never turns on brush or vacuum.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import protocol as p
from .drive import Bumped, DriveAborted, Driver
from .mapper import OCCUPIED_THRESHOLD, OccupancyMap

_LOGGER = logging.getLogger(__name__)

# SetMotor wheel distance per degree of turn in place: on a D7, 190 mm on each
# wheel turned it 90.0 +-0.3 deg (the wheels report ~193 mm)
MM_PER_DEG = 190.0 / 90.0
PLAN_CELL_M = 0.09
ROBOT_RADIUS_M = 0.17
CLEARANCE_M = 0.06            # extra room kept from walls when planning
STEP_M = 0.5                  # longest straight piece driven before rescanning
MIN_STEP_M = 0.05
MIN_GOAL_M = 0.35             # frontiers closer than this are seen already
BACK_OFF_MM = 80
MAX_BUMPS_PER_GOAL = 2
TIME_LIMIT_S = 40 * 60
MAX_STEPS = 500
MIN_BATTERY = 25
BATTERY_CHECK_EVERY = 15
DOCK_OUT_MAX_MM = 300
DOCK_OUT_MIN_MM = 120
# The planner trusts a single look: the map's own thresholds want a few scans
# agreeing before a cell counts as wall or floor, which suits cleans that pass
# by many times, not a robot that stops and looks once
PLAN_OCCUPIED = 0.5
PLAN_FREE = -0.3


def wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


@dataclass
class Frame:
    """Rigid transform from odometry (what GetRobotPos says) to map coordinates."""

    dx: float = 0.0
    dy: float = 0.0
    dtheta: float = 0.0

    def apply(self, pose: p.Pose) -> p.Pose:
        c, s = math.cos(math.radians(self.dtheta)), math.sin(math.radians(self.dtheta))
        return p.Pose(c * pose.x - s * pose.y + self.dx, s * pose.x + c * pose.y + self.dy,
                      (pose.theta + self.dtheta) % 360.0, pose.time)

    @classmethod
    def anchor(cls, odom: p.Pose, map_pose: p.Pose) -> "Frame":
        """The frame that puts this odometry reading at this map pose."""
        dtheta = wrap(map_pose.theta - odom.theta)
        c, s = math.cos(math.radians(dtheta)), math.sin(math.radians(dtheta))
        return cls(map_pose.x - (c * odom.x - s * odom.y), map_pose.y - (s * odom.x + c * odom.y), dtheta)


def match_scan(occ: OccupancyMap, scan: list[p.ScanPoint], guess: p.Pose,
               xy_range: float = 0.06, xy_step: float = 0.02,
               th_range: float = 3.0, th_step: float = 1.0) -> p.Pose:
    """Nudge a pose so the scan's hits land on walls already in the map."""
    cfg = occ.config
    pts = [pt for pt in scan if pt.error == 0 and cfg.min_range_m * 1000 <= pt.dist_mm <= cfg.max_range_m * 1000]
    if len(pts) < 40 or int((occ.grid >= OCCUPIED_THRESHOLD).sum()) < 150:
        return guess
    angles = np.radians([pt.angle_deg for pt in pts]) * (-1 if cfg.lds_clockwise else 1)
    angles += math.radians(cfg.lds_angle_offset_deg)
    dists = np.array([pt.dist_mm for pt in pts]) / 1000.0
    weights = np.clip(occ.grid, 0, None)
    offsets = np.arange(-xy_range, xy_range + 1e-9, xy_step)

    def score(theta: float, dx: np.ndarray, dy: np.ndarray) -> np.ndarray:
        h = math.radians(theta)
        sx = guess.x + cfg.lds_offset_m * math.cos(h)
        sy = guess.y + cfg.lds_offset_m * math.sin(h)
        hx = sx + dists * np.cos(angles + h)
        hy = sy + dists * np.sin(angles + h)
        col, row = occ.world_to_cell(hx[None, :] + dx[:, None], hy[None, :] + dy[:, None])
        ok = (col >= 0) & (col < occ.cells) & (row >= 0) & (row < occ.cells)
        vals = np.where(ok, weights[np.clip(row, 0, occ.cells - 1), np.clip(col, 0, occ.cells - 1)], 0)
        return vals.sum(axis=1)

    base = float(score(guess.theta, np.zeros(1), np.zeros(1))[0])
    gx, gy = np.meshgrid(offsets, offsets)
    gx, gy = gx.ravel(), gy.ravel()
    best = (base, 0.0, 0.0, 0.0)
    for dth in np.arange(-th_range, th_range + 1e-9, th_step):
        scores = score(guess.theta + dth, gx, gy)
        i = int(np.argmax(scores))
        if scores[i] > best[0]:
            best = (float(scores[i]), float(gx[i]), float(gy[i]), float(dth))
    # Only move when the fit is clearly better, so a featureless corridor
    # doesn't make the pose wander
    if best[0] < base * 1.05 + 1.0:
        return guess
    return p.Pose(guess.x + best[1], guess.y + best[2], (guess.theta + best[3]) % 360.0, guess.time)


class Planner:
    """Path planning on a coarse grid made from the occupancy map."""

    def __init__(self, occ: OccupancyMap, blocked: list[tuple[float, float]]) -> None:
        self.occ = occ
        res = occ.config.resolution_m
        self.k = max(1, int(round(PLAN_CELL_M / res)))
        bounds = occ.known_bounds(margin=self.k * 4)
        if bounds is None:
            raise ValueError("empty map")
        c0, r0, c1, r1 = bounds
        c1 = c0 + (c1 - c0) // self.k * self.k
        r1 = r0 + (r1 - r0) // self.k * self.k
        self.c0, self.r0 = c0, r0
        g = occ.grid[r0:r1, c0:c1]
        h, w = g.shape[0] // self.k, g.shape[1] // self.k
        blocks = g.reshape(h, self.k, w, self.k)
        occupied = (blocks >= PLAN_OCCUPIED).any(axis=(1, 3))
        free = (blocks <= PLAN_FREE).any(axis=(1, 3)) & ~occupied
        self.free = free
        for bx, by in blocked:
            cell = self.to_cell(bx, by)
            if cell is not None:
                occupied[cell] = True
                free[cell] = False
        self.unknown = ~free & ~occupied
        reach = int(math.ceil((ROBOT_RADIUS_M + CLEARANCE_M) / (self.k * res)))
        near_wall = np.zeros_like(occupied)
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                if dr * dr + dc * dc <= reach * reach:
                    near_wall |= _shift(occupied, dr, dc)
        self.safe = free & ~near_wall

    @property
    def shape(self) -> tuple[int, int]:
        return self.free.shape

    def to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col, row = self.occ.world_to_cell(x, y)
        r, c = (int(row) - self.r0) // self.k, (int(col) - self.c0) // self.k
        if 0 <= r < self.shape[0] and 0 <= c < self.shape[1]:
            return r, c
        return None

    def to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        r, c = cell
        return self.occ.cell_to_world(self.c0 + (c + 0.5) * self.k, self.r0 + (r + 0.5) * self.k)

    def start_cells(self, x: float, y: float, search_m: float = 0.45) -> list[tuple[int, int]]:
        """Safe cells to start a search from: the robot's own, or the nearest
        ones if it stands closer to a wall than the planner likes."""
        cell = self.to_cell(x, y)
        if cell is None:
            return []
        if self.safe[cell]:
            return [cell]
        reach = int(search_m / (self.k * self.occ.config.resolution_m))
        r0, c0 = cell
        found = []
        for r in range(max(0, r0 - reach), min(self.shape[0], r0 + reach + 1)):
            for c in range(max(0, c0 - reach), min(self.shape[1], c0 + reach + 1)):
                if self.safe[r, c]:
                    found.append(((r - r0) ** 2 + (c - c0) ** 2, (r, c)))
        found.sort()
        return [cell for _, cell in found[:1]]

    def bfs(self, starts: list[tuple[int, int]]) -> dict[tuple[int, int], tuple[int, int] | None]:
        parent: dict[tuple[int, int], tuple[int, int] | None] = {s: None for s in starts}
        queue = deque(starts)
        h, w = self.shape
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and (nr, nc) not in parent and self.safe[nr, nc]:
                    if dr and dc and not (self.safe[r, nc] and self.safe[nr, c]):
                        continue  # no cutting corners
                    parent[(nr, nc)] = (r, c)
                    queue.append((nr, nc))
        return parent

    def is_frontier(self, cell: tuple[int, int]) -> bool:
        r, c = cell
        h, w = self.shape
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and self.unknown[nr, nc]:
                    return True
        return False

    def nearest_frontier(self, x: float, y: float, skip: list[tuple[float, float]]) -> list[tuple[float, float]] | None:
        """Path (world points) to the nearest reachable frontier, or None."""
        starts = self.start_cells(x, y)
        if not starts:
            return None
        parent = self.bfs(starts)
        # BFS order is distance order, so the first good frontier is the nearest
        for cell in parent:
            wx, wy = self.to_world(cell)
            if math.dist((wx, wy), (x, y)) < MIN_GOAL_M or not self.is_frontier(cell):
                continue
            if any(math.dist((wx, wy), s) < 0.4 for s in skip):
                continue
            return self._path(parent, cell, x, y)
        return None

    def path_to(self, x: float, y: float, gx: float, gy: float) -> list[tuple[float, float]] | None:
        starts = self.start_cells(x, y)
        goals = self.start_cells(gx, gy, search_m=0.35)
        if not starts or not goals:
            return None
        parent = self.bfs(starts)
        if goals[0] not in parent:
            return None
        return self._path(parent, goals[0], x, y)

    def _path(self, parent, cell, x, y) -> list[tuple[float, float]]:
        cells = []
        while cell is not None:
            cells.append(cell)
            cell = parent[cell]
        return [self.to_world(c) for c in reversed(cells)]

    def line_is_safe(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        steps = max(2, int(math.dist(a, b) / (self.k * self.occ.config.resolution_m * 0.5)))
        for i in range(1, steps + 1):
            t = i / steps
            cell = self.to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if cell is None or not self.safe[cell]:
                return False
        return True


def _shift(a: np.ndarray, dr: int, dc: int) -> np.ndarray:
    out = np.zeros_like(a)
    h, w = a.shape
    out[max(dr, 0):h + min(dr, 0), max(dc, 0):w + min(dc, 0)] = a[max(-dr, 0):h - max(dr, 0), max(-dc, 0):w - max(dc, 0)]
    return out


class Explorer:
    def __init__(self, driver: Driver, occ: OccupancyMap, on_update=lambda: None) -> None:
        self.driver = driver
        self.link = driver.link
        self.occ = occ
        self.on_update = on_update
        self.frame = Frame()
        self.pose = p.Pose(0.0, 0.0, 0.0, 0.0)
        self.stop_requested = False
        self.go_home_requested = False
        self.status = "starting"
        self.blocked: list[tuple[float, float]] = []   # things the bumper found but lidar didn't
        self.skip: list[tuple[float, float]] = []      # frontiers we gave up on
        self.steps = 0
        self.docked: bool | None = None
        self.error: str | None = None
        # How far out of the dock it drove at the start: known to be clear,
        # so that's where it lines up to reverse back in
        self.out_m = DOCK_OUT_MAX_MM / 1000

    # -- pose and scans --------------------------------------------------------

    async def _odom(self) -> p.Pose:
        pose = p.parse_pose(await self.link.command("GetRobotPos Raw"))
        if pose is None:
            raise DriveAborted("no position from the robot")
        return pose

    async def observe(self) -> list[p.ScanPoint]:
        """Read pose and a scan while standing still, correct drift, update the map."""
        odom = await self._odom()
        scan, _ = p.parse_lds_scan(await self.link.command("GetLDSScan", timeout=8))
        guess = self.frame.apply(odom)
        matched = match_scan(self.occ, scan, guess)
        if matched is not guess:
            self.frame = Frame.anchor(odom, matched)
        self.pose = matched
        self.occ.integrate(matched, scan, matched)
        self.steps += 1
        self.on_update()
        return scan

    # -- motion ----------------------------------------------------------------

    async def turn_to(self, heading: float, scan: list[p.ScanPoint]) -> list[p.ScanPoint]:
        """Turn in place to a map heading. Odometry over- or under-counts long
        turns a little, so after a big one the pose is re-matched to the map
        and the rest corrected. Returns a scan taken at the final heading."""
        for _ in range(3):
            delta = wrap(heading - self.pose.theta)
            if abs(delta) < 2.0:
                break
            mm = int(round(delta * MM_PER_DEG))
            await self.driver.move(-mm, mm, scan)
            if abs(delta) > 20:
                scan = await self.observe()
            else:
                self.pose = self.frame.apply(await self._odom())
        return scan

    async def forward(self, dist_m: float, scan: list[p.ScanPoint], check: bool = True) -> None:
        mm = int(round(dist_m * 1000))
        if abs(mm) < 10:
            return
        await self.driver.move(mm, mm, scan, check_clearance=check)
        self.pose = self.frame.apply(await self._odom())

    async def back_off(self, scan: list[p.ScanPoint]) -> None:
        """After a bump: note the obstacle, then reverse away from it."""
        h = math.radians(self.pose.theta)
        self.blocked.append((self.pose.x + 0.25 * math.cos(h), self.pose.y + 0.25 * math.sin(h)))
        await self.driver.move(-BACK_OFF_MM, -BACK_OFF_MM, scan, check_clearance=False, backing_off=True)
        self.pose = self.frame.apply(await self._odom())

    async def follow(self, path: list[tuple[float, float]], planner: Planner, scan: list[p.ScanPoint]) -> None:
        """Drive the first straight, safe piece of a path (at most STEP_M)."""
        here = (self.pose.x, self.pose.y)
        target = path[1] if len(path) > 1 else path[0]
        for point in path[2:]:
            if math.dist(here, point) > STEP_M or not planner.line_is_safe(here, point):
                break
            target = point
        dist = math.dist(here, target)
        if dist < MIN_STEP_M:
            return
        heading = math.degrees(math.atan2(target[1] - here[1], target[0] - here[0]))
        # The clearance check uses a scan taken facing the new way
        scan = await self.turn_to(heading, scan)
        await self.forward(min(dist, STEP_M), scan)

    # -- the whole run ---------------------------------------------------------

    async def run(self) -> None:
        start = time.monotonic()
        await self.driver.enter()
        try:
            # Odometry doesn't always restart at zero; the dock is (0, 0, 0) in the map
            self.frame = Frame.anchor(await self._odom(), p.Pose(0.0, 0.0, 0.0, 0.0))
            scan = await self.observe()
            await self._leave_dock(scan)
            self.status = "exploring"
            bumps: dict[tuple[float, float], int] = {}
            stuck = 0
            while not (self.stop_requested or self.go_home_requested):
                if self.steps >= MAX_STEPS:
                    _LOGGER.info("Explore: step limit reached")
                    break
                if time.monotonic() - start > TIME_LIMIT_S:
                    _LOGGER.info("Explore: time limit reached")
                    break
                if self.steps % BATTERY_CHECK_EVERY == 0 and await self._battery_low():
                    break
                scan = await self.observe()
                planner = Planner(self.occ, self.blocked)
                path = planner.nearest_frontier(self.pose.x, self.pose.y, self.skip)
                if path is None:
                    _LOGGER.info("Explore: nothing left to explore after %d steps", self.steps)
                    break
                goal = path[-1]
                before = (self.pose.x, self.pose.y)
                try:
                    await self.follow(path, planner, scan)
                    # A goal we keep heading for without getting anywhere is skipped
                    stuck = stuck + 1 if math.dist(before, (self.pose.x, self.pose.y)) < MIN_STEP_M else 0
                    if stuck >= 3:
                        self.skip.append(goal)
                        stuck = 0
                except Bumped as err:
                    key = (round(goal[0], 1), round(goal[1], 1))
                    bumps[key] = bumps.get(key, 0) + 1
                    _LOGGER.info("Explore: %s on the way to %s, backing off", err, key)
                    if bumps[key] >= MAX_BUMPS_PER_GOAL:
                        self.skip.append(goal)
                    await self.back_off(scan)
                except DriveAborted as err:
                    if "free at" not in str(err):
                        raise
                    # Lidar says it's tighter than the map thought: try elsewhere
                    _LOGGER.info("Explore: %s, skipping that goal", err)
                    self.skip.append(goal)
            if not self.stop_requested:
                self.status = "returning"
                await self._return_to_dock()
        except DriveAborted as err:
            self.error = str(err)
            _LOGGER.warning("Explore stopped: %s", err)
        finally:
            await self.driver.leave()
        self.status = "done" if not self.error else f"stopped: {self.error}"

    async def _battery_low(self) -> bool:
        charger = p.parse_charger(await self.link.command("GetCharger"))
        if charger.battery_percent is not None and charger.battery_percent < MIN_BATTERY:
            _LOGGER.info("Explore: battery at %d%%, going back", charger.battery_percent)
            return True
        return False

    async def _leave_dock(self, scan: list[p.ScanPoint]) -> None:
        free = p.corridor_clearance_mm(scan, forward=True)
        out_mm = DOCK_OUT_MAX_MM if free is None else min(DOCK_OUT_MAX_MM, free - 250)
        if out_mm < DOCK_OUT_MIN_MM:
            raise DriveAborted(f"only {free} mm free in front of the dock")
        await self.forward(out_mm / 1000, scan)
        self.out_m = out_mm / 1000

    async def _return_to_dock(self) -> None:
        """Drive to a point in front of the dock, face away from it and reverse in."""
        approach = (self.out_m, 0.0)
        for _ in range(40):
            scan = await self.observe()
            if math.dist((self.pose.x, self.pose.y), approach) < 0.08:
                break
            planner = Planner(self.occ, self.blocked)
            path = planner.path_to(self.pose.x, self.pose.y, *approach)
            if path is None or len(path) < 2:
                # The planner keeps away from walls, so the last bit near the
                # dock goes straight, watched by the bumpers
                break
            try:
                await self.follow(path, planner, scan)
            except Bumped:
                await self.back_off(scan)
        scan = await self.observe()
        # Get onto the dock's axis, then face away from the dock and reverse in.
        # The last stretch is too close to the dock for the lidar clearance
        # check; the bumpers still watch it.
        dist = math.dist((self.pose.x, self.pose.y), approach)
        if dist >= MIN_STEP_M:
            scan = await self.turn_to(math.degrees(math.atan2(-self.pose.y, approach[0] - self.pose.x)), scan)
            await self.forward(dist, scan, check=False)
            scan = await self.observe()
        # Two passes: the second turn fixes what the first one's odometry got wrong
        for _ in range(2):
            scan = await self.turn_to(math.degrees(math.atan2(self.pose.y, self.pose.x)), scan)
            scan = await self.observe()
        await self.forward(-math.hypot(self.pose.x, self.pose.y), scan, check=False)
        self.occ.update_pose(self.pose)
        self.on_update()
        for _ in range(3):
            await asyncio.sleep(1.5)
            charger = p.parse_charger(await self.link.command("GetCharger"))
            self.docked = charger.external_power
            if self.docked:
                return
            await self.driver.move(-15, -15, scan, check_clearance=False, backing_off=True)
        _LOGGER.warning("Explore: not on the dock after reversing in")
