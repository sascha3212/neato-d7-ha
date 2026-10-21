"""Draws the occupancy map as a PNG for Home Assistant.

The grid is cleaned up (lone speckles removed, pinholes in the floor filled),
then floor and walls are upsampled as soft masks and re-thresholded, which
gives smooth anti-aliased outlines instead of blocky cells. The path, dock and
robot are drawn on a 2x canvas and scaled down for clean edges.
"""

from __future__ import annotations

import io
import math
from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from .mapper import FREE_THRESHOLD, OCCUPIED_THRESHOLD, OccupancyMap

FLOOR = (221, 233, 245)
GRID = (200, 216, 233)
WALL = (38, 52, 70)
WALL_SHADOW = (0, 0, 0)
PATH = (64, 132, 222)
ROBOT = (240, 128, 48)
DOCK = (52, 168, 96)
WHITE = (255, 255, 255)

SUPERSAMPLE = 2


@dataclass
class RenderedMap:
    png: bytes
    width: int
    height: int
    # Calibration for lovelace-xiaomi-vacuum-map-card: robot coordinates in
    # millimetres (the same units the add-on accepts for zones) -> pixels.
    calibration_points: list[dict]


def _neighbours(mask: np.ndarray) -> np.ndarray:
    """Number of set cells in each cell's 3x3 neighbourhood (itself excluded)."""
    m = np.pad(mask.astype(np.uint8), 1)
    total = sum(
        m[1 + dr: m.shape[0] - 1 + dr, 1 + dc: m.shape[1] - 1 + dc]
        for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc
    )
    return total


def clean_masks(grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (floor, wall) boolean masks with noise removed."""
    wall = grid >= OCCUPIED_THRESHOLD
    floor = grid <= FREE_THRESHOLD
    # Lone wall speckles (a person's leg, a lidar glitch) go away
    wall &= _neighbours(wall) >= 2
    # Hits that landed behind a wall (noise, reflections) aren't drawn: keep
    # only wall cells within two cells of observed floor
    near_floor = floor.copy()
    for _ in range(2):
        near_floor |= _neighbours(near_floor) > 0
    wall &= near_floor
    # Unknown or weak cells mostly surrounded by floor become floor (pinholes)
    for _ in range(2):
        floor |= ~wall & (_neighbours(floor) >= 5)
    # Floor right next to a wall is drawn under it, so outlines have no gaps
    floor |= wall
    return floor, wall


def _soft(mask: np.ndarray, size: tuple[int, int], blur: float, edge: float = 0.5) -> Image.Image:
    """Upsample a 0/1 mask smoothly and sharpen it back into an anti-aliased shape."""
    img = Image.fromarray((mask * 255).astype(np.uint8), "L").resize(size, Image.BILINEAR)
    if blur:
        img = img.filter(ImageFilter.GaussianBlur(blur))
    a = np.asarray(img, dtype=np.float32) / 255.0
    a = np.clip((a - edge) * 4.0 + 0.5, 0.0, 1.0)
    return Image.fromarray((a * 255).astype(np.uint8), "L")


def _chaikin(points: list[tuple[float, float]], rounds: int = 2) -> list[tuple[float, float]]:
    for _ in range(rounds):
        if len(points) < 3:
            return points
        out = [points[0]]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            out.append((0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1))
            out.append((0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1))
        out.append(points[-1])
        points = out
    return points


def render(occ: OccupancyMap, px_per_m: int = 100) -> RenderedMap | None:
    res = occ.config.resolution_m
    bounds = occ.known_bounds(margin=int(round(0.3 / res)))
    if bounds is None:
        return None
    c0, r0, c1, r1 = bounds
    floor_m, wall_m = clean_masks(occ.grid[r0:r1, c0:c1])
    scale = px_per_m * res  # output pixels per cell
    width = max(1, int(round((c1 - c0) * scale)))
    height = max(1, int(round((r1 - r0) * scale)))
    size = (width, height)

    # Soft masks: blur scales with the cell size so walls stay one even weight
    floor_a = _soft(floor_m, size, blur=scale * 0.35, edge=0.45)
    wall_a = _soft(wall_m, size, blur=scale * 0.8, edge=0.3)

    img = Image.new("RGBA", size, (0, 0, 0, 0))
    img.paste(Image.new("RGBA", size, FLOOR + (255,)), mask=floor_a)

    def to_px(x: float, y: float, k: float = 1.0) -> tuple[float, float]:
        half = occ.cells / 2
        col = x / res + half - c0
        row = half - y / res - r0
        return col * scale * k, row * scale * k

    # Faint 1 m grid on the floor, anchored at the dock
    grid_layer = Image.new("RGBA", size, (0, 0, 0, 0))
    gd = ImageDraw.Draw(grid_layer)
    x_min, y_max = occ.cell_to_world(c0, r0)
    x_max, y_min = occ.cell_to_world(c1, r1)
    for gx in range(math.ceil(x_min), math.floor(x_max) + 1):
        px, _ = to_px(gx, 0)
        gd.line([(px, 0), (px, height)], fill=GRID + (255,), width=1)
    for gy in range(math.ceil(y_min), math.floor(y_max) + 1):
        _, py = to_px(0, gy)
        gd.line([(0, py), (width, py)], fill=GRID + (255,), width=1)
    img.paste(grid_layer, mask=Image.fromarray(
        np.minimum(np.asarray(grid_layer.getchannel("A")), np.asarray(floor_a)), "L"))

    # Walls with a soft shadow for depth
    shadow = wall_a.filter(ImageFilter.GaussianBlur(max(1.5, px_per_m * 0.03)))
    shadow = shadow.point(lambda v: int(v * 0.35))
    shadow_layer = Image.new("RGBA", size, WALL_SHADOW + (255,))
    offset = Image.new("L", size, 0)
    offset.paste(shadow, (1, 2))
    img.paste(shadow_layer, mask=Image.fromarray(
        np.minimum(np.asarray(offset), np.asarray(floor_a)), "L"))
    img.paste(Image.new("RGBA", size, WALL + (255,)), mask=wall_a)

    # Path, dock and robot on a supersampled layer for smooth edges
    k = SUPERSAMPLE
    over = Image.new("RGBA", (width * k, height * k), (0, 0, 0, 0))
    od = ImageDraw.Draw(over)
    if len(occ.path) > 1:
        pts = _chaikin([to_px(x, y, k) for x, y in occ.path])
        od.line(pts, fill=PATH + (170,), width=max(2, int(px_per_m * 0.025 * k)), joint="curve")

    dx, dy = to_px(0.0, 0.0, k)
    r = px_per_m * 0.09 * k
    od.rounded_rectangle([dx - r, dy - r * 0.7, dx + r, dy + r * 0.7], radius=r * 0.3,
                         fill=DOCK + (255,), outline=WHITE + (255,), width=max(1, k * 2))

    if occ.robot is not None:
        rx, ry = to_px(occ.robot.x, occ.robot.y, k)
        r = px_per_m * 0.17 * k  # a D7 is about 34 cm wide
        od.ellipse([rx - r, ry - r + k * 3, rx + r, ry + r + k * 3], fill=(0, 0, 0, 60))
        od.ellipse([rx - r, ry - r, rx + r, ry + r], fill=ROBOT + (255,),
                   outline=WHITE + (255,), width=max(2, k * 3))
        t = math.radians(occ.robot.theta)
        tip = (rx + 0.7 * r * math.cos(t), ry - 0.7 * r * math.sin(t))
        left = (rx + 0.25 * r * math.cos(t + 2.4), ry - 0.25 * r * math.sin(t + 2.4))
        right = (rx + 0.25 * r * math.cos(t - 2.4), ry - 0.25 * r * math.sin(t - 2.4))
        od.polygon([tip, left, right], fill=WHITE + (255,))

    img = Image.alpha_composite(img, over.resize(size, Image.LANCZOS))

    buf = io.BytesIO()
    img.save(buf, "PNG", optimize=True)

    # Three non-collinear reference points (metres -> mm on the vacuum side)
    calibration = []
    for x, y in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]:
        px, py = to_px(x, y)
        calibration.append({
            "vacuum": {"x": round(x * 1000), "y": round(y * 1000)},
            "map": {"x": round(px, 2), "y": round(py, 2)},
        })
    return RenderedMap(buf.getvalue(), width, height, calibration)
