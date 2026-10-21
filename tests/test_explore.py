import asyncio
import math

import numpy as np
import pytest

from neato import connection, drive, explore
from neato import protocol as p
from neato.connection import RobotLink
from neato.mapper import MapperConfig, OccupancyMap
from neato.render import render
from tests.fake_robot import FakeRobot


@pytest.fixture
def fast(monkeypatch):
    monkeypatch.setattr(connection, "INTER_COMMAND_DELAY_S", 0.0)
    monkeypatch.setattr(drive, "SPEED_MM_S", 100000)
    monkeypatch.setattr(drive, "MOVE_SETTLE_S", 0.0)
    monkeypatch.setattr(drive, "WATCH_INTERVAL_S", 0.0)
    real_sleep = asyncio.sleep

    async def quick(delay, *args):
        await real_sleep(0, *args)

    monkeypatch.setattr(drive.asyncio, "sleep", quick)


def run_explore(robot, before=None):
    async def scenario():
        server = await asyncio.start_server(robot.handle, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        link = RobotLink("127.0.0.1", port)
        occ = OccupancyMap(MapperConfig())
        explorer = explore.Explorer(drive.Driver(link), occ)
        if before:
            before(explorer)
        try:
            await asyncio.wait_for(explorer.run(), 120)
        finally:
            server.close()
        return explorer, occ

    return asyncio.run(scenario())


def free_area_m2(occ):
    return float(occ.free().sum()) * occ.config.resolution_m ** 2


def test_explores_both_rooms_and_docks(fast):
    robot = FakeRobot()
    explorer, occ = run_explore(robot)
    assert explorer.error is None, explorer.error
    # The flat is 8 x 6 m with a sofa; most of it should be seen, both rooms
    assert free_area_m2(occ) > 35
    col, row = occ.world_to_cell(6.0, 0.0)
    assert occ.grid[row, col] < 0, "second room not mapped"
    assert explorer.docked
    assert robot.on_dock()
    assert not robot.test_mode
    assert not any("Vacuum" in c or "Brush" in c for c in robot.motor_log)
    assert render(occ) is not None


def test_bumps_into_what_lidar_cannot_see_and_carries_on(fast):
    robot = FakeRobot()
    robot.hidden = [(1.6, -0.4, 1.6, 0.4)]  # right in the robot's way out
    explorer, occ = run_explore(robot)
    assert explorer.error is None, explorer.error
    assert robot.bumps >= 1
    assert explorer.blocked
    assert free_area_m2(occ) > 35
    assert explorer.docked


def test_something_close_in_front_of_the_dock(fast):
    # Like Alexander's home: something ~45 cm ahead of the dock, a bit to the side
    robot = FakeRobot(extra_walls=[(0.45, 0.12, 0.45, 0.4), (0.45, 0.4, 0.6, 0.4), (0.6, 0.4, 0.6, 0.12)])
    explorer, occ = run_explore(robot)
    assert explorer.error is None, explorer.error
    assert explorer.out_m < 0.3
    assert explorer.docked


def test_scan_matching_corrects_turn_drift(fast):
    robot = FakeRobot()
    robot.turn_error = 1.03  # odometry over-reads every turn by 3%
    explorer, occ = run_explore(robot)
    assert explorer.error is None, explorer.error
    x, y, theta = robot.true_pose
    est = explorer.pose
    assert math.hypot(est.x - x, est.y - y) < 0.1
    assert abs((est.theta - theta + 180) % 360 - 180) < 3


def test_stop_request_leaves_testmode(fast):
    robot = FakeRobot()

    def stop_soon(explorer):
        original = explorer.observe

        async def observe():
            if explorer.steps >= 3:
                explorer.stop_requested = True
            return await original()

        explorer.observe = observe

    explorer, _ = run_explore(robot, stop_soon)
    assert explorer.status == "done"
    assert not robot.test_mode
    assert not robot.on_dock()  # stop means stop where it is


def test_frame_anchor_round_trip():
    odom = p.Pose(1.0, 2.0, 350.0, 0)
    frame = explore.Frame.anchor(odom, p.Pose(0.0, 0.0, 0.0, 0))
    out = frame.apply(odom)
    assert abs(out.x) < 1e-9 and abs(out.y) < 1e-9 and abs(explore.wrap(out.theta)) < 1e-9
    moved = frame.apply(p.Pose(1.0 + math.cos(math.radians(350)), 2.0 + math.sin(math.radians(350)), 350.0, 0))
    assert abs(moved.x - 1.0) < 1e-9 and abs(moved.y) < 1e-9
