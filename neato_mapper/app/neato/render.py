"""Draws the occupancy map as a PNG for Home Assistant."""

from __future__ import annotations

import io
import math
from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw

from .mapper import OccupancyMap

BACKGROUND = (0, 0, 0, 0)  # unknown = transparent, the dashboard shows through
FREE = (210, 228, 245, 255)
WALL = (45, 62, 80, 255)
PATH = (70, 140, 230, 200)
ROBOT = (235, 120, 40, 255)
DOCK = (60, 170, 90, 255)


@dataclass
class RenderedMap:
    png: bytes
    width: int
    height: int
    # Calibration for lovelace-xiaomi-vacuum-map-card: robot coordinates in
    # millimetres (the same units the add-on accepts for zones) -> pixels.
    calibration_points: list[dict]


def render(occ: OccupancyMap, scale: int = 4) -> RenderedMap | None:
    bounds = occ.known_bounds()
    if bounds is None:
        return None
    c0, r0, c1, r1 = bounds
    sub = occ.grid[r0:r1, c0:c1]

    rgba = np.zeros(sub.shape + (4,), dtype=np.uint8)
    rgba[:] = BACKGROUND
    rgba[sub <= -1.0] = FREE
    rgba[sub >= 1.5] = WALL
    img = Image.fromarray(rgba, "RGBA").resize(
        (sub.shape[1] * scale, sub.shape[0] * scale), Image.NEAREST
    )
    draw = ImageDraw.Draw(img)

    def to_px(x: float, y: float) -> tuple[float, float]:
        res = occ.config.resolution_m
        half = occ.cells / 2
        col = x / res + half - c0
        row = half - y / res - r0
        return col * scale, row * scale

    if len(occ.path) > 1:
        draw.line([to_px(x, y) for x, y in occ.path], fill=PATH, width=max(1, scale // 2))

    dx, dy = to_px(0.0, 0.0)
    r = scale * 2
    draw.rectangle([dx - r, dy - r, dx + r, dy + r], fill=DOCK)

    if occ.robot is not None:
        rx, ry = to_px(occ.robot.x, occ.robot.y)
        r = scale * 3.5
        draw.ellipse([rx - r, ry - r, rx + r, ry + r], fill=ROBOT, outline=(255, 255, 255, 255))
        t = math.radians(occ.robot.theta)
        draw.line([rx, ry, rx + 1.6 * r * math.cos(t), ry - 1.6 * r * math.sin(t)],
                  fill=(255, 255, 255, 255), width=max(1, scale // 2))

    buf = io.BytesIO()
    img.save(buf, "PNG", optimize=True)

    # Three non-collinear reference points (metres -> mm on the vacuum side)
    refs = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    calibration = []
    for x, y in refs:
        px, py = to_px(x, y)
        calibration.append({
            "vacuum": {"x": round(x * 1000), "y": round(y * 1000)},
            "map": {"x": round(px, 2), "y": round(py, 2)},
        })
    return RenderedMap(buf.getvalue(), img.width, img.height, calibration)
