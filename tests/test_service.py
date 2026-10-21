import asyncio
import json

import pytest

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
            await wait_for(lambda: service.map.scans_used >= 40)
            assert ha.published["map/image"].startswith(b"\x89PNG")
            attrs = json.loads(ha.published["map/attributes"])
            assert len(attrs["calibration_points"]) == 3

            ha.on_command(ha.topic("send_command"), json.dumps({"command": "raw", "params": {"cmd": "GetVersion"}}))
            await wait_for(lambda: "raw/attributes" in ha.published)
            assert "BotVacD7Connected" in json.loads(ha.published["raw/attributes"])["response"]

            ha.on_command(ha.topic("command"), "stop")
            await wait_for(lambda: json.loads(ha.published["state"])["state"] == "docked")
            await wait_for(lambda: (tmp_path / "map.npz").exists())
        finally:
            task.cancel()
            server.close()

    asyncio.run(scenario())
