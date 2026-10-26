"""Main loop: poll the robot, build the map, keep Home Assistant up to date."""

from __future__ import annotations

import asyncio
import json
import logging
import time
from datetime import datetime, timezone
from dataclasses import dataclass, field
from pathlib import Path

from . import protocol as p
from .connection import RobotLink
from .drive import Driver
from .explore import Explorer
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
        self.explorer: Explorer | None = None
        self.explore_task: asyncio.Task | None = None
        self.settings: dict[str, str] = {}
        self._last_settings = 0.0
        self.settings_file = opts.data_dir / "settings.json"
        self.navigation_mode = self._load_navigation_mode()
        # When the last clean started (ISO time), for "once a day" automations
        stored = self._load_stored()
        self.last_clean: str | None = stored.get("last_clean")
        self.last_clean_minutes: int | None = stored.get("last_clean_minutes")
        self.last_clean_type: str | None = stored.get("last_clean_type")
        self.spot_size = {k: int(stored.get(k, p.SPOT_SIZE_DEFAULT_CM)) for k in ("spot_width", "spot_height")}
        self._clean_started: float | None = None
        self.battery_temp: float | None = None
        self.warranty: dict[str, int] = {}
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
        await self._read_warranty()
        while True:
            await self._tick()

    # -- one loop pass -----------------------------------------------------

    async def _tick(self) -> None:
        while not self.commands.empty():
            await self._handle_command(*self.commands.get_nowait())

        if self.explore_task is not None:
            await self._explore_tick()
            return

        was_active = self._mapping_active()
        state = p.parse_state(await self.link.command("GetState"))
        if state is not None:
            self.state = state
        if "TESTMODE" in self.state.ui_state:
            # Left behind by an interrupted drive; the robot can't clean like this
            _LOGGER.warning("Robot is still in TestMode, switching it off")
            await self.link.command("SetMotor LWheelDisable RWheelDisable")
            await self.link.command("TestMode Off")
        self.error = p.parse_error(await self.link.command("GetErr"))
        now = time.monotonic()
        if self.charger is None or now - self._last_charger > CHARGER_INTERVAL_S:
            raw = await self.link.command("GetCharger")
            self.charger = p.parse_charger(raw)
            self.battery_temp = p.battery_temp_c(p.csv_values(raw))
            self._last_charger = now
        if not was_active and now - self._last_settings > CHARGER_INTERVAL_S:
            await self._read_settings()
        await self._track_clean()
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

    async def _track_clean(self) -> None:
        """Start time, duration and type of a clean, for automations and dashboards."""
        ui = self.state.ui_state
        if self._clean_started is None and p.is_cleaning(ui):
            self._clean_started = time.monotonic()
            self.last_clean = datetime.now(timezone.utc).isoformat(timespec="seconds")
            self.last_clean_type = p.clean_type(ui)
            self._save_stored()
        elif self._clean_started is not None and not p.in_clean_session(ui):
            self.last_clean_minutes = round((time.monotonic() - self._clean_started) / 60)
            self._clean_started = None
            _LOGGER.info("Clean (%s) took %d min", self.last_clean_type, self.last_clean_minutes)
            self._save_stored()
            await self._read_warranty()

    async def _read_warranty(self) -> None:
        warranty = p.parse_warranty(await self.link.command("GetWarranty"))
        if warranty:
            self.warranty = warranty

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
        if self.last_clean:
            self._publish_once("last_clean", self.last_clean)
        if self.last_clean_minutes is not None:
            self._publish_once("last_clean_duration", self.last_clean_minutes)
        if self.last_clean_type:
            self._publish_once("last_clean_type", self.last_clean_type)
        for key, value in self.spot_size.items():
            self._publish_once(f"settings/{key}", value)
        for suffix, label in (("filter_change_time", "Filter Change Time (seconds)"),
                              ("brush_change_time", "Brush Change Time (seconds)")):
            value = p.settings_number(self.settings, label)
            if value is not None:
                self._publish_once(suffix, round(value))
        if self.battery_temp is not None:
            self._publish_once("battery_temp", self.battery_temp)
        for suffix, key in (("battery_cycles", "CumulativeBatteryCycles"),
                            ("total_cleaning_time", "CumulativeCleaningTimeInSecs")):
            if key in self.warranty:
                value = self.warranty[key]
                self._publish_once(suffix, round(value / 3600, 1) if suffix == "total_cleaning_time" else value)
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

    # -- driving in TestMode ------------------------------------------------

    async def _can_drive(self, what: str) -> bool:
        if self.explore_task is None:
            # The cached reading can be a minute old, e.g. just after the
            # robot was put back on its dock by hand
            self.charger = p.parse_charger(await self.link.command("GetCharger"))
            self._last_charger = time.monotonic()
        ui = self.state.ui_state
        if self.explore_task is not None:
            reason = "already exploring"
        elif p.is_cleaning(ui) or p.is_paused(ui) or "DOCKING" in ui:
            reason = "robot is busy"
        elif self.error is not None:
            reason = f"robot reports {self.error.code} {self.error.message}"
        elif not (self.charger and self.charger.external_power):
            reason = "robot must start on its dock"
        else:
            return True
        _LOGGER.warning("%s refused: %s", what, reason)
        self.ha.publish("explore/state", f"refused: {reason}"[:250])
        return False

    async def _start_explore(self) -> None:
        if not await self._can_drive("Explore"):
            return
        _LOGGER.info("Explore: mapping the house without cleaning")
        self.map.start_session()

        def updated() -> None:
            self._map_dirty = True

        self.explorer = Explorer(Driver(self.link), self.map, on_update=updated)
        self.explore_task = asyncio.create_task(self.explorer.run())

    async def _explore_tick(self) -> None:
        """While exploring the explorer owns the serial link; just report."""
        assert self.explorer is not None and self.explore_task is not None
        ex = self.explorer
        self._publish_once("state", {"state": "returning" if ex.status == "returning" else "cleaning"})
        self._publish_once("robot_state", "exploring" if ex.status != "returning" else "returning")
        self._publish_once("explore/state", ex.status)
        self._maybe_publish_map()
        if not self.explore_task.done():
            await asyncio.sleep(STATUS_INTERVAL_S)
            return
        task, self.explore_task, self.explorer = self.explore_task, None, None
        err = task.exception()
        self.ha.publish("explore/attributes", {
            "steps": ex.steps, "docked": ex.docked, "bumper_obstacles": len(ex.blocked),
            "skipped_goals": len(ex.skip), "scans_in_map": self.map.scans_used,
            "error": str(err) if err else ex.error,
        })
        self._save_map()
        self._maybe_publish_map(force=True)
        self._published.clear()  # robot status is stale; republish on the next pass
        if err is not None:
            _LOGGER.error("Explore failed: %s", err)
            self._publish_once("explore/state", f"stopped: {err}"[:250])
            if isinstance(err, ConnectionError):
                raise err
        else:
            _LOGGER.info("Explore %s after %d steps, %s", ex.status, ex.steps,
                         "back on the dock" if ex.docked else "not on the dock")
            self._publish_once("explore/state", ex.status[:250])

    async def _drive_test(self) -> None:
        if not await self._can_drive("Drive test"):
            self.ha.publish("drive/state", "refused: see the Explore sensor for why")
            return
        _LOGGER.info("Drive test: out of the dock, a full circle and back in, in TestMode")
        self.ha.publish("drive/state", "running")
        log = await Driver(self.link).drive_test()
        result = log.as_dict()
        try:
            (self.opts.data_dir / "drive_test.json").write_text(json.dumps(result))
        except OSError as err:
            _LOGGER.warning("Could not save drive test: %s", err)
        if log.aborted:
            state = f"stopped: {log.aborted}"
        elif not log.docked:
            state = "done, but not back on the dock"
        else:
            state = "done"
        self.ha.publish("drive/state", state[:250])
        # Scans are too big for HA attributes; keep a summary there
        self.ha.publish("drive/attributes", {
            "aborted": log.aborted,
            "docked": log.docked,
            "steps": [{k: v for k, v in step.items() if k != "scan"} | {"scan_points": len(step["scan"])}
                      for step in log.steps],
        })
        self.ha.publish("drive/scans", result, retain=True)

    # -- robot settings ----------------------------------------------------

    async def _read_settings(self) -> None:
        settings = p.parse_user_settings(await self.link.command("GetUserSettings"))
        if settings:
            self.settings = settings
        self._last_settings = time.monotonic()

    def _load_stored(self) -> dict:
        try:
            data = json.loads(self.settings_file.read_text())
        except (OSError, ValueError):
            return {}
        return data if isinstance(data, dict) else {}

    def _load_navigation_mode(self) -> str:
        mode = self._load_stored().get("navigation_mode")
        return mode if mode in p.NAVIGATION_MODES else "Normal"

    def _save_stored(self) -> None:
        try:
            self.settings_file.write_text(json.dumps({
                "navigation_mode": self.navigation_mode,
                "last_clean": self.last_clean,
                "last_clean_minutes": self.last_clean_minutes,
                "last_clean_type": self.last_clean_type,
                **self.spot_size,
            }))
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
            self._save_stored()
            await self.link.command(f"SetNavigationMode {payload}")
        elif object_id in self.spot_size:
            try:
                value = round(float(payload))
            except ValueError:
                _LOGGER.warning("Bad spot size %r", payload)
                return
            self.spot_size[object_id] = max(p.SPOT_SIZE_MIN_CM, min(p.SPOT_SIZE_MAX_CM, value))
            self._save_stored()
            self._publish_status()
            return
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
        if self.explorer is not None:
            if cmd in ("stop", "pause"):
                self.explorer.stop_requested = True
            elif cmd == "return_to_base":
                self.explorer.go_home_requested = True
            else:
                _LOGGER.warning("Ignoring %s while exploring", cmd)
            return
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
        """vacuum.send_command: "reset_map", "clear_errors", "restart_robot", "explore",
        "drive_test", "spot_clean_size", "manual_cleaning",
        {"command": "manual_drive", "direction": "forward", "action": "down"|"up"}
        or {"command": "raw", "cmd": "GetVersion"}."""
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            data = {"command": payload}
        if not isinstance(data, dict):
            data = {"command": str(data)}
        command = data.get("command")
        # Home Assistant puts params next to "command"; older payloads nest them
        params = data.get("params") or {k: v for k, v in data.items() if k != "command"}
        if command == "clear_errors":
            _LOGGER.info("Clearing the robot's errors and alerts")
            await self.link.command("SetUIError clearall")
        elif command == "restart_robot":
            # What fang's PowerCycle button did: the robot reboots and is
            # silent for a while, so nothing waits for an answer
            _LOGGER.info("Restarting the robot")
            await self.link.command("TestMode On")
            await self.link.send_only("SetSystemMode PowerCycle")
            await self.link.close()
            raise ConnectionError("robot is restarting")
        elif command == "explore":
            await self._start_explore()
        elif self.explore_task is not None:
            _LOGGER.warning("Ignoring %s while exploring", command)
        elif command == "drive_test":
            await self._drive_test()
        elif command == "spot_clean_size":
            await self.link.command(f"SetNavigationMode {self.navigation_mode}")
            await self.link.command(p.spot_clean_command(self.spot_size["spot_width"], self.spot_size["spot_height"]))
        elif command == "manual_cleaning":
            await self._event(p.EVT_START_MANUAL, None)
        elif command == "manual_drive" and isinstance(params, dict):
            direction, action = params.get("direction"), params.get("action", "down")
            if direction not in p.MANUAL_DRIVE or action not in ("down", "up"):
                _LOGGER.warning("Bad manual drive %r", params)
                return
            await self._event(p.manual_drive_event(direction, action == "down"), None)
        elif command == "reset_map":
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
