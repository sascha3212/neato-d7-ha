"""Parsers and helpers for the Neato Botvac D3-D7 serial debug protocol.

Every response from the robot ends with a Ctrl-Z (0x1A) byte. Responses are
mostly CSV ("Label,Value") but the exact row order differs between firmware
versions, so everything here matches on labels instead of positions.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

TERMINATOR = b"\x1a"

EVT_START_HOUSE = "UIMGR_EVENT_SMARTAPP_START_HOUSE_CLEANING"
EVT_START_SPOT = "UIMGR_EVENT_SMARTAPP_START_SPOT_CLEANING"
EVT_PAUSE = "UIMGR_EVENT_SMARTAPP_PAUSE_CLEANING"
EVT_RESUME = "UIMGR_EVENT_SMARTAPP_RESUME_CLEANING"
EVT_STOP = "UIMGR_EVENT_SMARTAPP_STOP_CLEANING"
EVT_SEND_TO_BASE = "UIMGR_EVENT_SMARTAPP_SEND_TO_BASE"

# Error/alert code 200 (UI_ALERT_INVALID) means "nothing to report"
NO_ERROR_CODE = 200


@dataclass(frozen=True)
class Pose:
    """Robot pose as reported by GetRobotPos: metres and degrees (CCW from +X)."""

    x: float
    y: float
    theta: float
    time: float


@dataclass(frozen=True)
class ScanPoint:
    angle_deg: int
    dist_mm: int
    intensity: int
    error: int


@dataclass(frozen=True)
class RobotState:
    ui_state: str
    robot_state: str


@dataclass(frozen=True)
class Charger:
    battery_percent: int | None
    charging: bool
    external_power: bool


@dataclass(frozen=True)
class RobotError:
    code: int
    message: str


def strip_echo(raw: str, command: str) -> str:
    """Drop the echoed command line the robot prints before its answer."""
    lines = raw.replace("\r", "").split("\n")
    while lines and lines[0].strip() == "":
        lines.pop(0)
    if lines and lines[0].strip().lower() == command.strip().lower():
        lines.pop(0)
    return "\n".join(lines)


def csv_values(raw: str) -> dict[str, str]:
    """Map "Label,Value[,...]" lines to {label: value}, trailing commas removed."""
    values: dict[str, str] = {}
    for line in raw.replace("\r", "").split("\n"):
        line = line.strip()
        if "," not in line:
            continue
        key, value = line.split(",", 1)
        values[key.strip()] = value.strip().rstrip(",").strip()
    return values


def parse_state(raw: str) -> RobotState | None:
    ui = re.search(r"Current UI State is:\s*(\S+)", raw)
    robot = re.search(r"Current Robot State is:\s*(\S+)", raw)
    if not ui:
        return None
    return RobotState(ui.group(1), robot.group(1) if robot else "")


def parse_error(raw: str) -> RobotError | None:
    """Return the first real error/alert, or None when the robot is happy."""
    for match in re.finditer(r"(\d+)\s+-\s+(.*)", raw):
        code = int(match.group(1))
        if code > 0 and code != NO_ERROR_CODE:
            return RobotError(code, match.group(2).strip())
    return None


def _to_bool(value: str | None) -> bool:
    return (value or "").strip() in ("1", "true", "True", "ON", "On")


def parse_charger(raw: str) -> Charger:
    values = csv_values(raw)
    try:
        battery = int(values["FuelPercent"])
    except (KeyError, ValueError):
        battery = None
    return Charger(
        battery_percent=battery,
        charging=_to_bool(values.get("ChargingActive")),
        external_power=_to_bool(values.get("ExtPwrPresent")),
    )


_POSE_FIELD = r"{name}\s*=\s*(-?\d+(?:\.\d+)?)"


def parse_pose(raw: str) -> Pose | None:
    """Parse GetRobotPos output, e.g. "X=1.234 Y=-0.567 Theta=93.1 Time=12.0"."""
    found = {}
    for name in ("X", "Y", "Theta", "Time"):
        match = re.search(r"\b" + _POSE_FIELD.format(name=name), raw)
        if not match:
            return None
        found[name] = float(match.group(1))
    return Pose(found["X"], found["Y"], found["Theta"], found["Time"])


def parse_lds_scan(raw: str) -> tuple[list[ScanPoint], float]:
    """Parse GetLDSScan into (points, rotation speed in Hz).

    Angle 0 points straight ahead and angles grow counter-clockwise (to the
    robot's left), matching OpenNeato's verified rendering.
    """
    points: list[ScanPoint] = []
    speed = 0.0
    for line in raw.replace("\r", "").split("\n"):
        line = line.strip()
        if not line or line.startswith("Angle"):
            continue
        if line.startswith("ROTATION_SPEED"):
            try:
                speed = float(line.split(",", 1)[1])
            except (IndexError, ValueError):
                pass
            continue
        parts = line.split(",")
        if len(parts) < 4:
            continue
        try:
            angle, dist, intensity = int(parts[0]), int(parts[1]), int(parts[2])
            error = int(parts[3], 16)
        except ValueError:
            continue
        if 0 <= angle < 360:
            points.append(ScanPoint(angle, dist, intensity, error))
    return points, speed


def compute_skey(serial_value: str) -> str | None:
    """Compute the key the robot wants on SetEvent commands.

    `serial_value` is the "Serial Number" value from GetVersion with the label
    removed ("XXX,<12 hex MAC>,X"). Ported from OpenNeato (MIT licence), which
    derived it from the Brainslug project: RC4 with Neato's fixed seed, XORed
    with the MAC characters and hex encoded, plus one repeated character.
    """
    if "," not in serial_value:
        return None
    mac = serial_value.split(",", 1)[1][:12]
    if len(mac) != 12:
        return None

    seed = bytes([0x68, 0x36, 0x43, 0x58, 0x09, 0x09, 0x3A, 0x3C, 0x2A, 0x7B, 0x59])
    s = list(range(256))
    j = 0
    for i in range(256):
        j = (j + s[i] + seed[i % len(seed)]) & 0xFF
        s[i], s[j] = s[j], s[i]

    i = j = 0
    keystream = []
    for _ in range(12):
        i = (i + 1) & 0xFF
        j = (j + s[i]) & 0xFF
        s[i], s[j] = s[j], s[i]
        keystream.append(s[(s[i] + s[j]) & 0xFF])

    result = "".join(f"{k ^ ord(c):02x}" for k, c in zip(keystream, mac))
    return result + result[6]


def set_event(event: str, skey: str) -> str:
    return f"SetEvent event {event} SKey {skey}"


def is_cleaning(ui_state: str) -> bool:
    return any(
        tag in ui_state
        for tag in ("HOUSECLEANINGRUNNING", "SPOTCLEANINGRUNNING", "MANUALCLEANING")
    )


def is_paused(ui_state: str) -> bool:
    return "CLEANINGPAUSED" in ui_state


def ha_vacuum_state(ui_state: str, charger: Charger | None, error: RobotError | None) -> str:
    """Translate the robot's UI state into a Home Assistant vacuum state."""
    if error is not None and error.code >= 243:
        return "error"
    if is_cleaning(ui_state):
        return "cleaning"
    if is_paused(ui_state):
        return "paused"
    if "DOCKING" in ui_state or "RETURNING" in ui_state:
        return "returning"
    if charger is not None and charger.external_power:
        return "docked"
    return "idle"
