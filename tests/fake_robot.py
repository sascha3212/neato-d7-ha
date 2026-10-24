"""A pretend Neato D7 behind a pretend ESP32 bridge, for tests and demos.

Serves the serial protocol over TCP like the ESPHome stream server does. The
robot lives in a two-room flat, drives a lawnmower pattern when cleaning and
answers GetLDSScan by ray casting against the walls. Time is simulated: every
command moves the clock forward a little, so runs are fast and repeatable.

Run it standalone with:  python -m tests.fake_robot --port 6638
"""

from __future__ import annotations

import argparse
import asyncio
import math

import numpy as np

# Walls as segments (x1, y1, x2, y2) in metres; the dock is at (0, 0)
WALLS = [
    (-0.3, -3.0, 7.7, -3.0), (7.7, -3.0, 7.7, 3.0), (7.7, 3.0, -0.3, 3.0), (-0.3, 3.0, -0.3, -3.0),
    (3.7, -3.0, 3.7, 1.0), (3.7, 2.0, 3.7, 3.0),                                    # wall with a door
    (5.5, -2.5, 6.5, -2.5), (6.5, -2.5, 6.5, -1.8), (6.5, -1.8, 5.5, -1.8), (5.5, -1.8, 5.5, -2.5),  # sofa
]
SERIAL = "Serial Number,ABC12345DE,0123456789AB,D"


def lawnmower() -> list[tuple[float, float]]:
    pts = [(0.0, 0.0), (0.4, 0.0)]
    for i, y in enumerate(np.arange(-2.6, 2.7, 0.6)):
        xs = (0.2, 3.3) if i % 2 == 0 else (3.3, 0.2)
        pts += [(xs[0], y), (xs[1], y)]
    pts += [(3.3, 1.5), (4.2, 1.5)]
    for i, y in enumerate(np.arange(2.6, -2.7, -0.6)):
        xs = (4.1, 7.3) if i % 2 == 0 else (7.3, 4.1)
        if -2.9 < y < -1.4:
            xs = (4.1, 5.2) if i % 2 == 0 else (5.2, 4.1)
        pts += [(xs[0], y), (xs[1], y)]
    pts += [(4.2, 1.5), (3.3, 1.5), (0.4, 0.0), (0.0, 0.0)]
    return pts


class FakeRobot:
    def __init__(self, speed: float = 0.3, extra_walls=()) -> None:
        self.walls = WALLS + list(extra_walls)
        self.route = lawnmower()
        self.speed = speed
        self.clock = 0.0
        self.cleaning = False
        self.paused = False
        self.travel = 0.0
        self.user_settings = {
            "ClickSounds": "OFF", "LED": "ON", "Wall Enable": "ON", "Eco Mode": "ON",
            "IntenseClean": "OFF", "Melody Sounds": "OFF", "Warning Sounds": "ON", "Bin Full Detect": "ON",
        }
        self.navigation_mode = "Normal"
        self.sounds: list[str] = []
        self.test_mode = False
        self.lds_on = False
        self.bumper = False
        self.wheels_mm = [0.0, 0.0]
        self.motor_log: list[str] = []
        # Driven by hand in TestMode: where it really is, and what odometry says
        self.true_pose: list[float] | None = None
        self.odom: list[float] = [0.0, 0.0, 0.0]
        self.turn_error = 1.0     # odometry over/under-reads turns by this factor
        self.bumps = 0
        # Obstacles the lidar can't see (low chair rungs, a pet bowl): only the
        # bumper finds them. Segments like WALLS.
        self.hidden: list[tuple[float, float, float, float]] = []
        self.rng = np.random.default_rng(7)
        seg = np.array(self.walls)
        self._a = seg[:, :2]
        self._d = seg[:, 2:] - seg[:, :2]

    # -- motion ------------------------------------------------------------

    def pose(self) -> tuple[float, float, float]:
        if self.true_pose is not None:
            return tuple(self.true_pose)
        left = self.travel
        for (x0, y0), (x1, y1) in zip(self.route, self.route[1:]):
            length = math.dist((x0, y0), (x1, y1))
            if length == 0:
                continue
            heading = math.degrees(math.atan2(y1 - y0, x1 - x0))
            if left <= length:
                f = left / length
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), heading
            left -= length
        return 0.0, 0.0, 0.0

    def wall_distance(self, x: float, y: float) -> float:
        seg = np.array(self.walls + self.hidden)
        a, d = seg[:, :2], seg[:, 2:] - seg[:, :2]
        t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / (d ** 2).sum(axis=1), 0, 1)
        return float(np.hypot(a[:, 0] + t * d[:, 0] - x, a[:, 1] + t * d[:, 1] - y).min())

    def drive(self, left_mm: float, right_mm: float) -> None:
        """Move like the wheels would, stopping at walls with the bumper pressed."""
        if self.true_pose is None:
            self.true_pose = list(self.pose())
        x, y, theta = self.true_pose
        if abs(left_mm - right_mm) < 1e-6:
            dist = left_mm / 1000.0
            if dist < 0:
                self.bumper = False
            step = 0.01 if dist > 0 else -0.01
            done = 0.0
            while abs(done) < abs(dist) - 1e-9:
                nx = x + step * math.cos(math.radians(theta))
                ny = y + step * math.sin(math.radians(theta))
                if dist > 0 and self.wall_distance(nx, ny) < 0.17:
                    self.bumper = True
                    self.bumps += 1
                    break
                x, y, done = nx, ny, done + step
            o = math.radians(self.odom[2])
            self.odom[0] += done * math.cos(o)
            self.odom[1] += done * math.sin(o)
        else:
            dtheta = math.degrees(((right_mm - left_mm) / 2.0) / 121.0)  # SetMotor 190 mm turns a D7 90 deg
            theta = (theta + dtheta) % 360.0
            self.odom[2] = (self.odom[2] + dtheta * self.turn_error) % 360.0
        self.true_pose = [x, y, theta]

    def on_dock(self) -> bool:
        x, y, _ = self.pose()
        return not self.cleaning and math.hypot(x, y) < 0.03

    def route_length(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.route, self.route[1:]))

    def advance(self, dt: float) -> None:
        self.clock += dt
        if self.cleaning and not self.paused:
            self.travel += self.speed * dt
            if self.travel >= self.route_length():
                self.cleaning = False
                self.travel = 0.0

    # -- sensors -----------------------------------------------------------

    def scan(self) -> list[tuple[int, int]]:
        x, y, theta = self.pose()
        out = []
        for angle in range(360):
            a = math.radians(theta + angle)
            d = np.array([math.cos(a), math.sin(a)])
            # Ray/segment intersection for all walls at once
            denom = d[0] * self._d[:, 1] - d[1] * self._d[:, 0]
            ox, oy = self._a[:, 0] - x, self._a[:, 1] - y
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (ox * self._d[:, 1] - oy * self._d[:, 0]) / denom
                u = (ox * d[1] - oy * d[0]) / denom
            ok = (np.abs(denom) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
            if not ok.any() or self.rng.random() < 0.03:
                out.append((angle, 0))
                continue
            dist = float(t[ok].min()) + self.rng.normal(0, 0.01)
            out.append((angle, int(dist * 1000) if dist < 6.0 else 0))
        return out

    # -- protocol ----------------------------------------------------------

    def ui_state(self) -> str:
        if self.test_mode:
            return "UIMGR_STATE_TESTMODE"
        if self.cleaning:
            return "UIMGR_STATE_HOUSECLEANINGPAUSED" if self.paused else "UIMGR_STATE_HOUSECLEANINGRUNNING"
        return "UIMGR_STATE_IDLE"

    def answer(self, cmd: str) -> str:
        low = cmd.lower()
        if low == "getversion":
            self.advance(0.05)
            return f"Component,Major,Minor,Build\r\nProduct Model,BotVacD7Connected,,\r\n{SERIAL}\r\nSoftware,4,5,3\r\n"
        if low == "getstate":
            self.advance(0.05)
            return f"Current UI State is: {self.ui_state()}\r\nCurrent Robot State is: ST_C_Idle\r\n"
        if low == "geterr":
            self.advance(0.05)
            return ""
        if low == "getcharger":
            self.advance(0.05)
            ext = 1 if self.on_dock() else 0
            return f"Label,Value\r\nFuelPercent,87\r\nChargingActive,{ext}\r\nExtPwrPresent,{ext}\r\n"
        if low.startswith("getrobotpos"):
            self.advance(0.05)
            x, y, theta = self.odom if self.true_pose is not None else self.pose()
            return f"Pose: X={x:.3f} Y={y:.3f} Theta={theta:.1f} Time={self.clock:.2f}\r\n"
        if low == "getldsscan":
            # The last turret revolution, then ~0.5 s to send it at 115200 baud
            rows = "".join(f"{a},{d},{1200 if d else 0},{0 if d else 8035}\r\n" for a, d in self.scan())
            self.advance(0.5)
            return "AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX\r\n" + rows + "ROTATION_SPEED,5.00\r\n"
        if low.startswith("setevent"):
            self.advance(0.05)
            if "START_HOUSE" in cmd or "RESUME" in cmd:
                self.cleaning, self.paused = True, False
            elif "PAUSE" in cmd:
                self.paused = True
            elif "STOP" in cmd or "SEND_TO_BASE" in cmd:
                self.cleaning, self.paused, self.travel = False, False, 0.0
            return ""
        if low == "getusersettings":
            self.advance(0.05)
            return "".join(f"{k}, {v}\r\n" for k, v in self.user_settings.items()) + "Schedule is Disabled\r\n"
        if low.startswith("setusersettings"):
            self.advance(0.05)
            _, key, value = cmd.split()
            label = {"EcoMode": "Eco Mode", "WallEnable": "Wall Enable", "StealthLED": "LED"}.get(key, key)
            if key == "StealthLED":
                value = "OFF" if value == "ON" else "ON"
            self.user_settings[label] = value
            return ""
        if low.startswith("setnavigationmode"):
            self.navigation_mode = cmd.split()[1]
            return ""
        if low.startswith("playsound"):
            self.sounds.append(cmd)
            return ""
        if low.startswith("testmode"):
            self.test_mode = low.endswith("on")
            return ""
        if low.startswith("setldsrotation"):
            self.lds_on = low.endswith("on")
            return ""
        if low.startswith("setmotor"):
            self.motor_log.append(cmd)
            if not self.test_mode:
                return "TestMode must be on to use this command.\r\n"
            parts = cmd.split()
            if "LWheelDist" in parts:
                left = float(parts[parts.index("LWheelDist") + 1])
                right = float(parts[parts.index("RWheelDist") + 1])
                self.wheels_mm[0] += left
                self.wheels_mm[1] += right
                if not self.bumper or left < 0:
                    self.drive(left, right)
            return ""
        if low == "getdigitalsensors":
            bit = int(self.bumper)
            return f"Digital Sensor Name, Value\r\nSNSR_DC_JACK_IS_IN,0\r\nLSIDEBIT,0\r\nLFRONTBIT,{bit}\r\nRFRONTBIT,0\r\n"
        if low == "getanalogsensors":
            return "SensorName,Unit,Value\r\nWallSensor,mm,255,\r\nDropSensorLeft,mm,19,\r\nDropSensorRight,mm,20,\r\n"
        if low.startswith("getmotors"):
            return (f"Parameter,Value\r\nLeftWheel_PositionInMM,{self.wheels_mm[0]:.0f}\r\n"
                    f"RightWheel_PositionInMM,{self.wheels_mm[1]:.0f}\r\n")
        if low == "setuierror clearall":
            self.errors_cleared = getattr(self, "errors_cleared", 0) + 1
            return ""
        if low.startswith("setsystemmode"):
            self.restarts = getattr(self, "restarts", 0) + 1
            self.test_mode = False
            return None  # rebooting: no answer
        if low == "clean house":
            self.cleaning, self.paused = True, False
            return ""
        return f"Unknown Cmd: '{cmd}'\r\n"

    async def handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while line := await reader.readline():
                cmd = line.decode().strip()
                if not cmd:
                    continue
                reply = self.answer(cmd)
                if reply is None:
                    continue
                writer.write(f"{cmd}\r\n{reply}".encode() + b"\x1a")
                await writer.drain()
        except ConnectionError:
            pass
        finally:
            writer.close()


async def serve(port: int) -> None:
    robot = FakeRobot()
    server = await asyncio.start_server(robot.handle, "0.0.0.0", port)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=6638)
    asyncio.run(serve(parser.parse_args().port))
