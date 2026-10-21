"""Drive the robot directly in TestMode, with the add-on doing the safety checks.

In TestMode the robot's own bumper and cliff reactions are off, so every move
here is checked first (bumpers, lifted wheels, cliff sensors, lidar clearance)
and watched while it runs: any bumper or wheel-lift stops the wheels at once.
Brush, vacuum and side brush are never switched on.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field

from . import protocol as p
from .connection import RobotLink

_LOGGER = logging.getLogger(__name__)

SPEED_MM_S = 100
WATCH_INTERVAL_S = 0.3
# Cliff sensors read ~20 mm on a floor; much more means there is no floor
MAX_DROP_MM = 60
# Room needed in the travel direction beyond the move itself (robot radius + margin)
CLEARANCE_MARGIN_MM = 250
# How far the drive test goes out of the dock: as far as the lidar allows, up
# to the maximum, but not less than the minimum
OUT_MAX_MM = 300
OUT_MIN_MM = 120
# Wheel arc for a 90 degree turn in place (D7 wheelbase ~240 mm)
QUARTER_TURN_MM = 190


class DriveAborted(Exception):
    pass


@dataclass
class DriveLog:
    steps: list[dict] = field(default_factory=list)
    aborted: str | None = None
    docked: bool | None = None
    started: float = field(default_factory=time.time)

    def as_dict(self) -> dict:
        return {"started": self.started, "aborted": self.aborted, "docked": self.docked, "steps": self.steps}


class Driver:
    def __init__(self, link: RobotLink) -> None:
        self.link = link
        self.log = DriveLog()

    # -- readings ------------------------------------------------------------

    async def _scan(self) -> list[p.ScanPoint]:
        scan, _ = p.parse_lds_scan(await self.link.command("GetLDSScan", timeout=8))
        return scan

    async def _safety(self) -> None:
        digital = p.csv_values(await self.link.command("GetDigitalSensors"))
        if not any(name in digital for name in p.BUMPER_BITS):
            raise DriveAborted("can't read the bumpers, not driving")
        hit = p.bumpers_hit(digital)
        if hit:
            raise DriveAborted(f"bumper or wheel lift: {', '.join(hit)}")
        drops = p.drop_distances_mm(p.csv_values(await self.link.command("GetAnalogSensors")))
        if any(d > MAX_DROP_MM for d in drops):
            raise DriveAborted(f"cliff sensor reads {drops} mm")

    async def snapshot(self, label: str) -> list[p.ScanPoint]:
        smooth = await self.link.command("GetRobotPos Smooth")
        raw = await self.link.command("GetRobotPos Raw")
        wheels = p.wheel_positions_mm(p.csv_values(await self.link.command("GetMotors LeftWheel RightWheel")))
        scan = await self._scan()
        pose_s, pose_r = p.parse_pose(smooth), p.parse_pose(raw)
        self.log.steps.append({
            "step": label,
            "pose_smooth": [pose_s.x, pose_s.y, pose_s.theta] if pose_s else smooth.strip()[:120],
            "pose_raw": [pose_r.x, pose_r.y, pose_r.theta] if pose_r else raw.strip()[:120],
            "wheels_mm": list(wheels) if wheels else None,
            "scan": [[pt.angle_deg, pt.dist_mm] for pt in scan if pt.dist_mm and pt.error == 0],
            "scan_errors": sum(1 for pt in scan if pt.error),
        })
        _LOGGER.info("Drive %s: smooth=%s raw=%s wheels=%s scan=%d points",
                     label, self.log.steps[-1]["pose_smooth"], self.log.steps[-1]["pose_raw"],
                     wheels, len(self.log.steps[-1]["scan"]))
        return scan

    # -- motion --------------------------------------------------------------

    async def stop_wheels(self) -> None:
        await self.link.command("SetMotor LWheelDisable RWheelDisable")

    async def move(self, left_mm: int, right_mm: int, scan: list[p.ScanPoint],
                   check_clearance: bool = True) -> None:
        """One checked move. Straight moves need lidar clearance that way."""
        await self._safety()
        if check_clearance and left_mm == right_mm:
            direction = 0 if left_mm > 0 else 180
            free = p.clearance_mm(scan, direction)
            if free is not None and free < abs(left_mm) + CLEARANCE_MARGIN_MM:
                raise DriveAborted(f"only {free} mm free at {direction} deg")
        await self.link.command("SetMotor LWheelEnable RWheelEnable")
        await self.link.command(f"SetMotor LWheelDist {left_mm} RWheelDist {right_mm} Speed {SPEED_MM_S}")
        deadline = time.monotonic() + max(abs(left_mm), abs(right_mm)) / SPEED_MM_S + 1.5
        try:
            while time.monotonic() < deadline:
                await asyncio.sleep(WATCH_INTERVAL_S)
                await self._safety()
        except DriveAborted:
            await self.stop_wheels()
            raise

    # -- sessions ------------------------------------------------------------

    async def enter(self) -> None:
        await self.link.command("TestMode On")
        await self.link.command("SetLDSRotation On")
        await asyncio.sleep(4)  # let the turret reach speed

    async def leave(self) -> None:
        for cmd in ("SetMotor LWheelDisable RWheelDisable", "SetLDSRotation Off", "TestMode Off"):
            try:
                await self.link.command(cmd)
            except ConnectionError as err:
                _LOGGER.error("Could not send %r while leaving TestMode: %s", cmd, err)

    async def drive_test(self) -> DriveLog:
        """Drive up to 30 cm out of the dock, turn a full circle in four steps, reverse
        back in, and record everything on the way.

        Neatos back into their dock, so leaving it is a forward move.
        """
        await self.snapshot("docked_before")
        await self.enter()
        try:
            scan = await self.snapshot("testmode_docked")
            free = p.clearance_mm(scan, 0)
            out_mm = OUT_MAX_MM if free is None else min(OUT_MAX_MM, free - CLEARANCE_MARGIN_MM)
            if out_mm < OUT_MIN_MM:
                raise DriveAborted(f"only {free} mm free in front of the dock")
            await self.move(out_mm, out_mm, scan)
            scan = await self.snapshot("off_dock")
            for i in range(4):
                await self.move(-QUARTER_TURN_MM, QUARTER_TURN_MM, scan)
                scan = await self.snapshot(f"turn_{i + 1}")
            # The robot only finds its dock by itself during a clean, so it
            # reverses the same distance back in. The dock is right behind it, so no
            # lidar clearance check here; the bumpers still stop it.
            await self.move(-out_mm, -out_mm, scan, check_clearance=False)
            await asyncio.sleep(2)
            self.log.docked = p.parse_charger(await self.link.command("GetCharger")).external_power
            await self.snapshot("back_on_dock")
        except DriveAborted as err:
            _LOGGER.warning("Drive test stopped: %s", err)
            self.log.aborted = str(err)
        finally:
            await self.leave()
        return self.log
