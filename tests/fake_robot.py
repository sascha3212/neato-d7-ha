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
    def __init__(self, speed: float = 0.3) -> None:
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
        self.rng = np.random.default_rng(7)
        seg = np.array(WALLS)
        self._a = seg[:, :2]
        self._d = seg[:, 2:] - seg[:, :2]

    # -- motion ------------------------------------------------------------

    def pose(self) -> tuple[float, float, float]:
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
            ext = 0 if self.cleaning else 1
            return f"Label,Value\r\nFuelPercent,87\r\nChargingActive,{ext}\r\nExtPwrPresent,{ext}\r\n"
        if low.startswith("getrobotpos"):
            self.advance(0.05)
            x, y, theta = self.pose()
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
                writer.write(f"{cmd}\r\n{self.answer(cmd)}".encode() + b"\x1a")
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
