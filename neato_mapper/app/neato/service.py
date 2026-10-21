"""Main loop: poll the robot, build the map, keep Home Assistant up to date."""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import protocol as p
from .connection import RobotLink
from .ha_mqtt import HaMqtt
from .mapper import MapperConfig, OccupancyMap
from .render import render

_LOGGER = logging.getLogger(__name__)

STATUS_INTERVAL_S = 2.0
CHARGER_INTERVAL_S = 60.0
MAP_PUBLISH_INTERVAL_S = 5.0
MAP_SAVE_INTERVAL_S = 60.0
# A lidar scan is ~5 KB at 115200 baud. Asking for them back to back while the
# robot cleans made a D7 (4.4.0) stop answering on serial, so scans are spaced
# out, and spaced further after any failure during a clean.
SCAN_INTERVAL_S = 4.0
SCAN_INTERVAL_MAX_S = 30.0


@dataclass
class Options:
    bridge_host: str
    bridge_port: int = 6638
    name: str = "Neato"
    node_id: str = "neato"
    data_dir: Path = Path("/data")
    mqtt_host: str = "core-mosquitto"
    mqtt_port: int = 1883
    mqtt_username: str | None = None
    mqtt_password: str | None = None
    mapper: MapperConfig = field(default_factory=MapperConfig)
    map_px_per_m: int = 100


class NeatoService:
    def __init__(self, opts: Options, version: str = "dev") -> None:
        self.opts = opts
        self.link = RobotLink(opts.bridge_host, opts.bridge_port)
        self.map = OccupancyMap(opts.mapper)
        self.map_file = opts.data_dir / "map.npz"
        self.ha = HaMqtt(opts.node_id, opts.name, version)
        self.commands: asyncio.Queue[tuple[str, str]] = asyncio.Queue()
        self.skey: str | None = None
        self.state = p.RobotState("", "")
        self.error: p.RobotError | None = None
        self.charger: p.Charger | None = None
        self._last_charger = 0.0
        self._last_scan = 0.0
        self.scan_interval = SCAN_INTERVAL_S
        self.settings: dict[str, str] = {}
        self._last_settings = 0.0
        self.settings_file = opts.data_dir / "settings.json"
        self.navigation_mode = self._load_navigation_mode()
        self._last_map_publish = 0.0
        self._last_map_save = time.monotonic()
        self._map_dirty = True
        self._published: dict[str, str] = {}

    # -- start up ----------------------------------------------------------

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        self.ha.on_command = lambda topic, payload: loop.call_soon_threadsafe(
            self.commands.put_nowait, (topic, payload)
        )
        self.ha.connect(self.opts.mqtt_host, self.opts.mqtt_port,
                        self.opts.mqtt_username, self.opts.mqtt_password)
        if self.map.load(self.map_file):
            _LOGGER.info("Loaded saved map (%d scans)", self.map.scans_used)

        backoff = 2.0
        while True:
            try:
                await self._session()
                backoff = 2.0
            except ConnectionError as err:
                _LOGGER.warning("Robot unreachable: %s (retry in %.0fs)", err, backoff)
            except OSError as err:
                _LOGGER.warning("Bridge unreachable: %s (retry in %.0fs)", err, backoff)
            self.ha.set_available(False)
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 60.0)

    async def _session(self) -> None:
        await self.link.connect()
        version = p.csv_values(await self.link.command("GetVersion"))
        self.skey = p.compute_skey(version.get("Serial Number", ""))
        _LOGGER.info("Robot %s, software %s, SetEvent key %s",
                     p.product_model(version), p.software_version(version),
                     "ok" if self.skey else "unavailable (using plain Clean commands)")
        self.ha.set_available(True)
        self._published.clear()
        self._last_settings = 0.0
        while True:
            await self._tick()

    # -- one loop pass -----------------------------------------------------

    async def _tick(self) -> None:
        while not self.commands.empty():
            await self._handle_command(*self.commands.get_nowait())

        was_active = self._mapping_active()
        state = p.parse_state(await self.link.command("GetState"))
        if state is not None:
            self.state = state
        self.error = p.parse_error(await self.link.command("GetErr"))
        now = time.monotonic()
        if self.charger is None or now - self._last_charger > CHARGER_INTERVAL_S:
            self.charger = p.parse_charger(await self.link.command("GetCharger"))
            self._last_charger = now
        if not was_active and now - self._last_settings > CHARGER_INTERVAL_S:
            await self._read_settings()
        self._publish_status()

        active = self._mapping_active()
        if active and not was_active:
            _LOGGER.info("Cleaning started, recording a new path")
            self.map.start_session()
        if active and now - self._last_scan >= self.scan_interval:
            self._last_scan = now
            await self._map_step()
        elif was_active and not active:
            _LOGGER.info("Cleaning finished: %d scans in map", self.map.scans_used)
            self._save_map()

        self._maybe_publish_map(force=was_active and not active)
        await asyncio.sleep(STATUS_INTERVAL_S)

    def _mapping_active(self) -> bool:
        ui = self.state.ui_state
        return p.is_cleaning(ui) or "DOCKING" in ui

    async def _map_step(self) -> None:
        try:
            await self._scan_once()
        except ConnectionError:
            self.scan_interval = min(self.scan_interval * 2, SCAN_INTERVAL_MAX_S)
            _LOGGER.warning("Scan failed during a clean, next scans every %.0fs", self.scan_interval)
            raise

    async def _scan_once(self) -> None:
        before = p.parse_pose(await self.link.command("GetRobotPos Smooth"))
        scan, _speed = p.parse_lds_scan(await self.link.command("GetLDSScan", timeout=8))
        after = p.parse_pose(await self.link.command("GetRobotPos Smooth"))
        if before is None or after is None:
            _LOGGER.debug("No pose from GetRobotPos, skipping scan")
            return
        self.map.integrate(before, scan, after)
        self._map_dirty = True

    # -- Home Assistant ----------------------------------------------------

    def _publish_once(self, suffix: str, value) -> None:
        text = json.dumps(value) if isinstance(value, (dict, list)) else str(value)
        if self._published.get(suffix) != text:
            self._published[suffix] = text
            self.ha.publish(suffix, text)

    def _publish_status(self) -> None:
        state = {"state": p.ha_vacuum_state(self.state.ui_state, self.charger, self.error)}
        if p.fan_speed(self.settings):
            state["fan_speed"] = p.fan_speed(self.settings)
        self._publish_once("state", state)
        self._publish_once("robot_state", self.state.ui_state.replace("UIMGR_STATE_", "").lower() or "unknown")
        self._publish_once("error", f"{self.error.code} {self.error.message}" if self.error else "none")
        if self.charger and self.charger.battery_percent is not None:
            self._publish_once("battery", self.charger.battery_percent)
        self._publish_once("settings/navigation_mode", self.navigation_mode)
        for object_id, _name, _key, label, _icon, _inverted in p.TOGGLE_SETTINGS:
            on = p.toggle_state(self.settings, label)
            if on is not None:
                self._publish_once(f"settings/{object_id}", "ON" if on else "OFF")
        self._publish_once("attributes", {
            "ui_state": self.state.ui_state,
            "robot_state": self.state.robot_state,
            "charging": bool(self.charger and self.charger.charging),
        })

    def _maybe_publish_map(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (not self._map_dirty or now - self._last_map_publish < MAP_PUBLISH_INTERVAL_S):
            return
        rendered = render(self.map, self.opts.map_px_per_m)
        self._last_map_publish = now
        self._map_dirty = False
        if rendered is None:
            return
        self.ha.publish("map/image", rendered.png)
        self.ha.publish("map/attributes", {
            "calibration_points": rendered.calibration_points,
            "resolution_m": self.map.config.resolution_m,
            "scans_used": self.map.scans_used,
            "scans_skipped": self.map.scans_skipped,
        })
        self._publish_once("map/scans", self.map.scans_used)
        if now - self._last_map_save > MAP_SAVE_INTERVAL_S:
            self._save_map()

    # -- robot settings ----------------------------------------------------

    async def _read_settings(self) -> None:
        settings = p.parse_user_settings(await self.link.command("GetUserSettings"))
        if settings:
            self.settings = settings
        self._last_settings = time.monotonic()

    def _load_navigation_mode(self) -> str:
        try:
            mode = json.loads(self.settings_file.read_text()).get("navigation_mode")
        except (OSError, ValueError, AttributeError):
            mode = None
        return mode if mode in p.NAVIGATION_MODES else "Normal"

    def _save_navigation_mode(self) -> None:
        try:
            self.settings_file.write_text(json.dumps({"navigation_mode": self.navigation_mode}))
        except OSError as err:
            _LOGGER.warning("Could not save settings: %s", err)

    async def _set_setting(self, object_id: str, payload: str) -> None:
        _LOGGER.info("Setting %s -> %s", object_id, payload)
        if object_id == "fan_speed":
            if payload not in p.FAN_SPEEDS:
                _LOGGER.warning("Unknown fan speed %r", payload)
                return
            await self.link.command(f"SetUserSettings EcoMode {'ON' if payload == 'eco' else 'OFF'}")
        elif object_id == "navigation_mode":
            if payload not in p.NAVIGATION_MODES:
                _LOGGER.warning("Unknown navigation mode %r", payload)
                return
            # The robot forgets this on restart, so it is also sent before every clean
            self.navigation_mode = payload
            self._save_navigation_mode()
            await self.link.command(f"SetNavigationMode {payload}")
        else:
            toggle = next((t for t in p.TOGGLE_SETTINGS if t[0] == object_id), None)
            if toggle is None or payload not in ("ON", "OFF"):
                _LOGGER.warning("Unknown setting %s=%r", object_id, payload)
                return
            _id, _name, key, _label, _icon, inverted = toggle
            await self.link.command(p.set_toggle(key, payload == "ON", inverted))
        await self._read_settings()
        self._publish_status()

    def _save_map(self) -> None:
        self.map.save(self.map_file)
        self._last_map_save = time.monotonic()

    # -- commands from Home Assistant -------------------------------------

    async def _handle_command(self, topic: str, payload: str) -> None:
        if topic == self.ha.topic("command"):
            await self._vacuum_command(payload.strip())
        elif topic == self.ha.topic("send_command"):
            await self._custom_command(payload.strip())
        elif topic.startswith(self.ha.topic("set/")):
            await self._set_setting(topic.rsplit("/", 1)[1], payload.strip())

    async def _event(self, event: str, fallback: str | None) -> None:
        if self.skey:
            await self.link.command(p.set_event(event, self.skey))
        elif fallback:
            await self.link.command(fallback)
        else:
            _LOGGER.warning("Cannot send %s without a SetEvent key", event)

    async def _vacuum_command(self, cmd: str) -> None:
        _LOGGER.info("Vacuum command: %s", cmd)
        ui = self.state.ui_state
        if cmd == "start":
            if p.is_paused(ui):
                await self._event(p.EVT_RESUME, "Clean House")
            else:
                await self.link.command(f"SetNavigationMode {self.navigation_mode}")
                await self._event(p.EVT_START_HOUSE, "Clean House")
        elif cmd == "pause":
            await self._event(p.EVT_PAUSE, None)
        elif cmd == "stop":
            await self._event(p.EVT_STOP, "Clean Stop")
        elif cmd == "return_to_base":
            await self._event(p.EVT_SEND_TO_BASE, None)
        elif cmd == "clean_spot":
            await self._event(p.EVT_START_SPOT, "Clean Spot")
        elif cmd == "locate":
            await self.link.command(f"PlaySound SoundID {p.LOCATE_SOUND_ID}")
        else:
            _LOGGER.warning("Unsupported vacuum command %r", cmd)

    async def _custom_command(self, payload: str) -> None:
        """vacuum.send_command: "reset_map", or {"command": "raw", "cmd": "GetVersion"}."""
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            data = {"command": payload}
        if not isinstance(data, dict):
            data = {"command": str(data)}
        command = data.get("command")
        # Home Assistant puts params next to "command"; older payloads nest them
        params = data.get("params") or {k: v for k, v in data.items() if k != "command"}
        if command == "reset_map":
            _LOGGER.info("Resetting the map")
            self.map.reset()
            self._save_map()
            self._map_dirty = True
            self._maybe_publish_map(force=True)
        elif command == "raw" and isinstance(params, dict) and params.get("cmd"):
            raw_cmd = str(params["cmd"]).strip()
            response = await self.link.command(raw_cmd, timeout=10)
            _LOGGER.info("Raw %r ->\n%s", raw_cmd, response)
            self.ha.publish("raw/state", raw_cmd[:250], retain=False)
            self.ha.publish("raw/attributes", {"command": raw_cmd, "response": response}, retain=False)
        else:
            _LOGGER.warning("Unknown send_command payload %r", payload)
