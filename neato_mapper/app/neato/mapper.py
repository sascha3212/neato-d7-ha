"""Occupancy grid built from lidar scans placed at the robot's own SLAM pose.

The robot already localizes itself while cleaning (GetRobotPos Smooth), so we
don't run SLAM here: each scan is projected from the reported pose and voted
into a log-odds grid. Repeated cleans sharpen the walls and wash out people
and chairs that moved.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .protocol import Pose, ScanPoint

# Log-odds update weights and clamp. Hits count more than misses so thin
# furniture legs survive, the clamp keeps the map able to change its mind.
HIT = 0.9
MISS = -0.4
CLAMP = 6.0
OCCUPIED_THRESHOLD = 1.5
FREE_THRESHOLD = -1.0


@dataclass
class MapperConfig:
    resolution_m: float = 0.05
    size_m: float = 40.0
    max_range_m: float = 5.0
    min_range_m: float = 0.12
    # Where the lidar sits relative to the robot centre (+ = forward), and a
    # rotation correction in case the turret's zero isn't exactly forward.
    lds_offset_m: float = 0.0
    lds_angle_offset_deg: float = 0.0
    # Mirror the scan (some firmware reports angles clockwise)
    lds_clockwise: bool = False
    # Drop a scan when, between the pose readings around it, the robot
    # turned more than this (smeared scan) or jumped implausibly far
    max_move_m: float = 0.6
    max_turn_deg: float = 15.0


@dataclass
class OccupancyMap:
    config: MapperConfig = field(default_factory=MapperConfig)
    grid: np.ndarray = field(init=False)
    path: list[tuple[float, float]] = field(default_factory=list)
    robot: Pose | None = None
    scans_used: int = 0
    scans_skipped: int = 0

    def __post_init__(self) -> None:
        cells = int(round(self.config.size_m / self.config.resolution_m))
        self.grid = np.zeros((cells, cells), dtype=np.float32)

    # -- coordinates -------------------------------------------------------

    @property
    def cells(self) -> int:
        return self.grid.shape[0]

    def world_to_cell(self, x: np.ndarray | float, y: np.ndarray | float):
        """World metres -> (column, row). The dock (0, 0) is the grid centre;
        rows grow downwards so the image has +Y pointing up."""
        half = self.cells / 2
        col = np.floor(np.asarray(x) / self.config.resolution_m + half).astype(int)
        row = np.floor(half - np.asarray(y) / self.config.resolution_m).astype(int)
        return col, row

    def cell_to_world(self, col: float, row: float) -> tuple[float, float]:
        half = self.cells / 2
        return (col - half) * self.config.resolution_m, (half - row) * self.config.resolution_m

    # -- updates -----------------------------------------------------------

    def update_pose(self, pose: Pose) -> None:
        self.robot = pose
        if not self.path or math.dist(self.path[-1], (pose.x, pose.y)) >= 0.05:
            self.path.append((pose.x, pose.y))

    def integrate(self, before: Pose, scan: list[ScanPoint], after: Pose) -> bool:
        """Add one scan, with pose readings taken right before and after it.

        The robot answers GetLDSScan with the last full turret revolution, so
        the scan was captured at about the `before` pose; sending ~5 KB at
        115200 baud then takes half a second. We place the scan at `before`
        and drop it when the robot was turning hard, which smears a scan.
        Returns False when the scan was dropped."""
        moved = math.dist((before.x, before.y), (after.x, after.y))
        turned = abs((after.theta - before.theta + 180) % 360 - 180)
        if moved > self.config.max_move_m or turned > self.config.max_turn_deg:
            self.scans_skipped += 1
            self.update_pose(after)
            return False

        cfg = self.config
        heading = math.radians(before.theta)
        sx = before.x + cfg.lds_offset_m * math.cos(heading)
        sy = before.y + cfg.lds_offset_m * math.sin(heading)

        pts = [p for p in scan if p.error == 0 and p.dist_mm > 0]
        if len(pts) < 20:
            self.scans_skipped += 1
            return False
        angles = np.radians([p.angle_deg for p in pts]).astype(np.float64)
        if cfg.lds_clockwise:
            angles = -angles
        angles += heading + math.radians(cfg.lds_angle_offset_deg)
        dists = np.array([p.dist_mm for p in pts], dtype=np.float64) / 1000.0
        keep = dists >= cfg.min_range_m
        angles, dists = angles[keep], dists[keep]

        hits = dists <= cfg.max_range_m
        free_len = np.minimum(dists, cfg.max_range_m)

        # Free space: sample every ray at half-cell steps up to just before the hit
        step = cfg.resolution_m / 2
        samples = np.arange(0.0, cfg.max_range_m, step)
        mask = samples[None, :] < (free_len[:, None] - cfg.resolution_m)
        fx = sx + samples[None, :] * np.cos(angles)[:, None]
        fy = sy + samples[None, :] * np.sin(angles)[:, None]
        fcol, frow = self.world_to_cell(fx[mask], fy[mask])
        free_flat = self._flat_in_bounds(fcol, frow)

        hx = sx + dists[hits] * np.cos(angles[hits])
        hy = sy + dists[hits] * np.sin(angles[hits])
        hcol, hrow = self.world_to_cell(hx, hy)
        hit_flat = self._flat_in_bounds(hcol, hrow)

        # A wall cell crossed by a neighbouring ray is still a wall
        free_flat = np.setdiff1d(free_flat, hit_flat, assume_unique=True)
        flat_grid = self.grid.reshape(-1)
        flat_grid[free_flat] += MISS
        flat_grid[hit_flat] += HIT
        np.clip(self.grid, -CLAMP, CLAMP, out=self.grid)

        self.update_pose(after)
        self.scans_used += 1
        return True

    def _flat_in_bounds(self, col: np.ndarray, row: np.ndarray) -> np.ndarray:
        ok = (col >= 0) & (col < self.cells) & (row >= 0) & (row < self.cells)
        return np.unique(row[ok] * self.cells + col[ok])

    def start_session(self) -> None:
        """A new clean starts a new path; the grid keeps what it learned."""
        self.path.clear()

    def reset(self) -> None:
        self.grid[:] = 0
        self.path.clear()
        self.robot = None
        self.scans_used = self.scans_skipped = 0

    # -- queries -----------------------------------------------------------

    def occupied(self) -> np.ndarray:
        return self.grid >= OCCUPIED_THRESHOLD

    def free(self) -> np.ndarray:
        return self.grid <= FREE_THRESHOLD

    def known_bounds(self, margin: int = 10) -> tuple[int, int, int, int] | None:
        """(col0, row0, col1, row1) around everything observed, or None."""
        known = np.abs(self.grid) > 0.01
        if not known.any():
            return None
        rows = np.where(known.any(axis=1))[0]
        cols = np.where(known.any(axis=0))[0]
        return (
            max(int(cols[0]) - margin, 0),
            max(int(rows[0]) - margin, 0),
            min(int(cols[-1]) + margin + 1, self.cells),
            min(int(rows[-1]) + margin + 1, self.cells),
        )

    # -- persistence -------------------------------------------------------

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp.npz")
        np.savez_compressed(
            tmp,
            grid=self.grid,
            resolution=self.config.resolution_m,
            scans_used=self.scans_used,
        )
        tmp.replace(path)

    def load(self, path: Path) -> bool:
        if not path.exists():
            return False
        data = np.load(path)
        if float(data["resolution"]) != self.config.resolution_m or data["grid"].shape != self.grid.shape:
            return False
        self.grid[:] = data["grid"]
        self.scans_used = int(data["scans_used"])
        return True
