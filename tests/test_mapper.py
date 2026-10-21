import numpy as np

from neato.mapper import MapperConfig, OccupancyMap
from neato.protocol import Pose, ScanPoint
from neato.render import render
from tests.fake_robot import FakeRobot


def run_clean(occ, robot, steps):
    robot.cleaning = True
    for _ in range(steps):
        robot.advance(0.05)
        x, y, t = robot.pose()
        before = Pose(x, y, t, robot.clock)
        scan = [ScanPoint(a, d, 1000, 0 if d else 1) for a, d in robot.scan()]
        robot.advance(0.5)
        x, y, t = robot.pose()
        occ.integrate(before, scan, Pose(x, y, t, robot.clock))


def is_wall_near(occ, x, y, tol_cells=2):
    col, row = occ.world_to_cell(x, y)
    window = occ.grid[row - tol_cells: row + tol_cells + 1, col - tol_cells: col + tol_cells + 1]
    return window.max() >= 1.5


def test_map_finds_walls_and_free_space():
    occ = OccupancyMap(MapperConfig())
    robot = FakeRobot(speed=0.3)
    run_clean(occ, robot, 600)
    assert occ.scans_used > 400
    # Outer walls and the dividing wall show up where they really are
    assert is_wall_near(occ, -0.3, 0.5)
    assert is_wall_near(occ, 2.0, 3.0)
    assert is_wall_near(occ, 3.7, -1.0)
    # The middle of the first room is free, the door gap is not a wall
    col, row = occ.world_to_cell(1.5, 0.5)
    assert occ.grid[row, col] < -1.0
    assert not is_wall_near(occ, 3.7, 1.5, tol_cells=1)


def test_skips_scans_taken_while_spinning():
    occ = OccupancyMap(MapperConfig())
    scan = [ScanPoint(a, 1500, 1000, 0) for a in range(360)]
    assert not occ.integrate(Pose(0, 0, 0, 0), scan, Pose(0, 0, 40, 0.2))
    assert occ.integrate(Pose(0, 0, 0, 0), scan, Pose(0.02, 0, 2, 0.2))


def test_render_and_calibration(tmp_path):
    occ = OccupancyMap(MapperConfig())
    run_clean(occ, FakeRobot(speed=0.3), 80)
    out = render(occ, px_per_m=100)
    assert out is not None and out.png.startswith(b"\x89PNG")
    origin, x1, y1 = out.calibration_points
    # 1 m is 100 px; +Y points up in the image
    assert abs((x1["map"]["x"] - origin["map"]["x"]) - 100) < 0.01
    assert abs((origin["map"]["y"] - y1["map"]["y"]) - 100) < 0.01

    occ.save(tmp_path / "map.npz")
    again = OccupancyMap(MapperConfig())
    assert again.load(tmp_path / "map.npz")
    assert np.array_equal(again.grid, occ.grid)
