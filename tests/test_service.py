import asyncio
import json

import pytest

from neato import connection
from neato import service as svc
from tests.fake_robot import FakeRobot


class RecordingHa:
    """Stands in for the MQTT side: remembers what would be published."""

    def __init__(self, node_id, name, version):
        self.base = f"neato_mapper/{node_id}"
        self.published = {}
        self.available = None
        self.on_command = lambda topic, payload: None

    def topic(self, suffix):
        return f"{self.base}/{suffix}"

    def connect(self, *args):
        pass

    def publish(self, suffix, payload, retain=True):
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        self.published[suffix] = payload

    def set_available(self, available):
        self.available = available


@pytest.fixture
def fast(monkeypatch):
    monkeypatch.setattr(svc, "HaMqtt", RecordingHa)
    monkeypatch.setattr(svc, "STATUS_INTERVAL_S", 0.01)
    monkeypatch.setattr(svc, "MAP_PUBLISH_INTERVAL_S", 0.0)
    monkeypatch.setattr(svc, "SCAN_INTERVAL_S", 0.0)
    monkeypatch.setattr(connection, "INTER_COMMAND_DELAY_S", 0.0)


async def wait_for(predicate, timeout=20.0):
    loop = asyncio.get_running_loop()
    end = loop.time() + timeout
    while not predicate():
        if loop.time() > end:
            raise AssertionError("timed out")
        await asyncio.sleep(0.02)


def test_clean_builds_and_publishes_map(fast, tmp_path):
    async def scenario():
        robot = FakeRobot(speed=0.3)
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: ha.available)
            await wait_for(lambda: "state" in ha.published)
            assert json.loads(ha.published["state"])["state"] == "docked"
            assert ha.published["battery"] == "87"
            assert service.skey == "b20e9dfade296f1210da702bf"

            ha.on_command(ha.topic("command"), "start")
            await wait_for(lambda: json.loads(ha.published["state"])["state"] == "cleaning")
            await wait_for(lambda: "last_clean" in ha.published)
            assert ha.published["last_clean"].startswith("20")
            assert json.loads((tmp_path / "settings.json").read_text())["last_clean"] == ha.published["last_clean"]
            await wait_for(lambda: service.map.scans_used >= 40)
            assert ha.published["map/image"].startswith(b"\x89PNG")
            attrs = json.loads(ha.published["map/attributes"])
            assert len(attrs["calibration_points"]) == 3

            ha.on_command(ha.topic("send_command"), json.dumps({"command": "raw", "params": {"cmd": "GetVersion"}}))
            await wait_for(lambda: "raw/attributes" in ha.published)
            assert "BotVacD7Connected" in json.loads(ha.published["raw/attributes"])["response"]
            # The shape Home Assistant actually sends
            ha.on_command(ha.topic("send_command"), json.dumps({"command": "raw", "cmd": "GetState"}))
            await wait_for(lambda: "GetState" == json.loads(ha.published["raw/attributes"])["command"])

            ha.on_command(ha.topic("command"), "stop")
            await wait_for(lambda: json.loads(ha.published["state"])["state"] == "docked")
            await wait_for(lambda: (tmp_path / "map.npz").exists())
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def test_settings_modes_and_locate(fast, tmp_path):
    async def scenario():
        robot = FakeRobot(speed=0.3)
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "settings/status_lights" in ha.published)
            assert json.loads(ha.published["state"])["fan_speed"] == "eco"
            assert ha.published["settings/edge_cleaning"] == "ON"
            assert ha.published["settings/button_sounds"] == "OFF"
            assert ha.published["settings/status_lights"] == "ON"
            assert ha.published["settings/navigation_mode"] == "Normal"

            ha.on_command(ha.topic("set/fan_speed"), "turbo")
            await wait_for(lambda: json.loads(ha.published["state"]).get("fan_speed") == "turbo")
            assert robot.user_settings["Eco Mode"] == "OFF"

            ha.on_command(ha.topic("set/status_lights"), "OFF")
            await wait_for(lambda: ha.published["settings/status_lights"] == "OFF")
            assert robot.user_settings["LED"] == "OFF"

            ha.on_command(ha.topic("set/intense_clean"), "ON")
            await wait_for(lambda: ha.published["settings/intense_clean"] == "ON")

            ha.on_command(ha.topic("set/navigation_mode"), "Deep")
            await wait_for(lambda: ha.published["settings/navigation_mode"] == "Deep")
            robot.navigation_mode = "Normal"  # the robot forgets it on restart
            ha.on_command(ha.topic("command"), "start")
            await wait_for(lambda: robot.cleaning)
            assert robot.navigation_mode == "Deep"
            assert json.loads((tmp_path / "settings.json").read_text())["navigation_mode"] == "Deep"

            ha.on_command(ha.topic("command"), "locate")
            await wait_for(lambda: robot.sounds)
            assert robot.sounds == ["PlaySound SoundID 19"]
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def test_no_finish_between_scans(fast, monkeypatch, tmp_path):
    """Loop passes between two scans must not count as the end of a clean."""
    monkeypatch.setattr(svc, "SCAN_INTERVAL_S", 0.5)

    async def scenario():
        robot = FakeRobot(speed=0.3)
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        saves = []
        service._save_map = lambda: saves.append(1)
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: ha.available)
            ha.on_command(ha.topic("command"), "start")
            await wait_for(lambda: service.map.scans_used >= 3)
            assert saves == []
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def run_drive_test(tmp_path, robot, monkeypatch):
    from neato import drive
    monkeypatch.setattr(drive.asyncio, "sleep", _fast_sleep)
    monkeypatch.setattr(drive, "SPEED_MM_S", 2000)  # moves are timed in wall-clock

    async def scenario():
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "state" in ha.published)
            ha.on_command(ha.topic("send_command"), "drive_test")
            await wait_for(lambda: ha.published.get("drive/state", "running") != "running")
            return ha
        finally:
            task.cancel()
            server.close()

    return asyncio.run(scenario())


_real_sleep = asyncio.sleep


async def _fast_sleep(delay, *args):
    await _real_sleep(min(delay, 0.01), *args)


def test_drive_test_moves_and_returns(fast, monkeypatch, tmp_path):
    robot = FakeRobot()
    ha = run_drive_test(tmp_path, robot, monkeypatch)
    assert ha.published["drive/state"] == "done"
    steps = json.loads(ha.published["drive/attributes"])["steps"]
    assert [s["step"] for s in steps][:3] == ["docked_before", "testmode_docked", "off_dock"]
    assert steps[2]["wheels_mm"] == [300.0, 300.0]
    assert steps[-1]["step"] == "back_on_dock"
    assert ha.published["drive/state"] == "done"  # charger contacts found again
    assert not robot.test_mode and not robot.lds_on
    assert not any("Vacuum" in c or "Brush" in c for c in robot.motor_log)
    assert (tmp_path / "drive_test.json").exists()


def test_drive_test_stops_on_bumper(fast, monkeypatch, tmp_path):
    robot = FakeRobot()
    robot.bumper = True
    ha = run_drive_test(tmp_path, robot, monkeypatch)
    assert ha.published["drive/state"].startswith("stopped: bumper")
    assert robot.wheels_mm == [0.0, 0.0]
    assert not robot.test_mode


def _serve_lines(answer):
    """A bridge that answers every line with answer(line)."""
    async def handle(reader, writer):
        while line := await reader.readline():
            writer.write(answer(line.decode().strip()))
            await writer.drain()
    return handle


def test_out_of_sync_answer_gives_up_after_resync(monkeypatch):
    from neato import connection
    from neato.connection import RobotLink
    monkeypatch.setattr(connection, "RESYNC_QUIET_S", 0.05)

    async def scenario():
        # Always the answer to something else
        server = await asyncio.start_server(
            _serve_lines(lambda cmd: b"GetState\r\nCurrent UI State is: X\r\n\x1a"), "127.0.0.1", 0)
        link = RobotLink("127.0.0.1", server.sockets[0].getsockname()[1])
        try:
            with pytest.raises(ConnectionError):
                await link.command("GetCharger")
            assert not link.connected
        finally:
            server.close()

    asyncio.run(scenario())


def test_leftover_answer_is_skipped(monkeypatch):
    from neato import connection
    from neato.connection import RobotLink
    monkeypatch.setattr(connection, "RESYNC_QUIET_S", 0.05)
    sent = []

    def answer(cmd):
        sent.append(cmd)
        reply = f"{cmd}\r\nFuelPercent,87\r\n".encode() + b"\x1a"
        if len(sent) == 1:  # the tail of an earlier lidar scan arrives first
            return b"358,1200,900,0\r\n359,1210,900,0\r\nROTATION_SPEED,5.00\r\n\x1a" + reply
        return reply

    async def scenario():
        server = await asyncio.start_server(_serve_lines(answer), "127.0.0.1", 0)
        link = RobotLink("127.0.0.1", server.sockets[0].getsockname()[1])
        try:
            assert "FuelPercent,87" in await link.command("GetCharger")
            assert "FuelPercent,87" in await link.command("GetCharger")  # and in step again
        finally:
            server.close()

    asyncio.run(scenario())


def test_explore_button(fast, monkeypatch, tmp_path):
    from neato import drive
    monkeypatch.setattr(drive.asyncio, "sleep", _fast_sleep)
    monkeypatch.setattr(drive, "SPEED_MM_S", 100000)
    monkeypatch.setattr(drive, "MOVE_SETTLE_S", 0.0)
    monkeypatch.setattr(drive, "WATCH_INTERVAL_S", 0.0)

    async def scenario():
        robot = FakeRobot()
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "battery" in ha.published)
            ha.on_command(ha.topic("send_command"), "explore")
            await wait_for(lambda: ha.published.get("robot_state") == "exploring")
            await wait_for(lambda: ha.published.get("explore/state") == "done", timeout=60)
            attrs = json.loads(ha.published["explore/attributes"])
            assert attrs["docked"] is True and attrs["steps"] > 10
            assert service.map.scans_used > 10
            assert (tmp_path / "map.npz").exists()
            assert robot.on_dock() and not robot.test_mode
            await wait_for(lambda: json.loads(ha.published.get("state", "{}")).get("state") == "docked")
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def test_explore_refused_off_dock(fast, tmp_path):
    async def scenario():
        robot = FakeRobot()
        robot.true_pose = [1.0, 0.0, 0.0]
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "battery" in ha.published)
            ha.on_command(ha.topic("send_command"), "explore")
            await wait_for(lambda: "explore/state" in ha.published)
            assert ha.published["explore/state"] == "refused: robot must start on its dock"
            assert not robot.test_mode
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def test_explore_reads_fresh_dock_state(fast, monkeypatch, tmp_path):
    """Put back on the dock by hand: the cached charger reading is stale."""
    from neato import drive
    monkeypatch.setattr(drive.asyncio, "sleep", _fast_sleep)
    monkeypatch.setattr(drive, "SPEED_MM_S", 100000)
    monkeypatch.setattr(drive, "MOVE_SETTLE_S", 0.0)
    monkeypatch.setattr(drive, "WATCH_INTERVAL_S", 0.0)

    async def scenario():
        robot = FakeRobot()
        robot.true_pose = [1.0, 0.0, 0.0]
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "battery" in ha.published)
            robot.true_pose = [0.0, 0.0, 0.0]  # carried back to the dock
            ha.on_command(ha.topic("send_command"), "explore")
            await wait_for(lambda: ha.published.get("robot_state") == "exploring")
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def test_clear_errors_and_restart(fast, tmp_path):
    async def scenario():
        robot = FakeRobot()
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "battery" in ha.published)
            ha.on_command(ha.topic("send_command"), "clear_errors")
            await wait_for(lambda: getattr(robot, "errors_cleared", 0) == 1)
            ha.on_command(ha.topic("send_command"), "restart_robot")
            await wait_for(lambda: getattr(robot, "restarts", 0) == 1)
            assert not robot.test_mode
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())


def test_fang_replacements(fast, tmp_path):
    """Manual drive, sized spot clean and the counters the old dashboards used."""
    async def scenario():
        robot = FakeRobot(speed=0.3)
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        service = svc.NeatoService(svc.Options(bridge_host="127.0.0.1", bridge_port=port, data_dir=tmp_path))
        task = asyncio.create_task(service.run())
        try:
            ha = service.ha
            await wait_for(lambda: "filter_change_time" in ha.published)
            assert ha.published["filter_change_time"] == "432000"
            assert ha.published["brush_change_time"] == "1296000"
            assert ha.published["battery_temp"] == "31.0"
            assert ha.published["battery_cycles"] == "503"
            assert ha.published["total_cleaning_time"] == str(round(0x1ce34f / 3600, 1))
            assert ha.published["settings/spot_width"] == "200"

            ha.on_command(ha.topic("set/spot_width"), "320")
            ha.on_command(ha.topic("set/spot_height"), "999")
            await wait_for(lambda: ha.published["settings/spot_height"] == "400")
            assert ha.published["settings/spot_width"] == "320"
            assert json.loads((tmp_path / "settings.json").read_text())["spot_width"] == 320
            ha.on_command(ha.topic("send_command"), "spot_clean_size")
            await wait_for(lambda: getattr(robot, "spot_cleans", None))
            assert robot.spot_cleans == ["Clean Spot Width 320 Height 400"]

            ha.on_command(ha.topic("send_command"), "manual_cleaning")
            ha.on_command(ha.topic("send_command"),
                          json.dumps({"command": "manual_drive", "direction": "arc_left", "action": "down"}))
            ha.on_command(ha.topic("send_command"),
                          json.dumps({"command": "manual_drive", "direction": "forward", "action": "up"}))
            await wait_for(lambda: len(getattr(robot, "events", [])) >= 3)
            assert robot.events == [
                "UIMGR_EVENT_SMARTAPP_START_MANUAL_CLEANING",
                "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_ARC_LEFT_DOWN",
                "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_FORWARD_UP",
            ]

            ha.on_command(ha.topic("command"), "start")
            await wait_for(lambda: ha.published.get("last_clean_type") == "house")
            ha.on_command(ha.topic("command"), "pause")
            await wait_for(lambda: robot.paused)
            ha.on_command(ha.topic("command"), "start")
            await wait_for(lambda: not robot.paused)
            ha.on_command(ha.topic("command"), "stop")
            await wait_for(lambda: "last_clean_duration" in ha.published)
            assert ha.published["last_clean_duration"] == "0"
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())
