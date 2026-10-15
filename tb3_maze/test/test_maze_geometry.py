"""Geometry / config consistency tests. Run without ROS:  python3 -m pytest tb3_maze/test"""

from pathlib import Path
import sys

import yaml

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from tb3_maze.maze_geometry import MazeLayout  # noqa: E402

LAYOUT = MazeLayout.load(PKG / 'config' / 'maze_layout.yaml')
GOALS = yaml.safe_load((PKG / 'config' / 'goals.yaml').read_text())['goals']


def test_all_cells_reachable():
    assert LAYOUT.unreachable_cells() == []


def test_spawn_is_world_origin():
    assert LAYOUT.cell_center(LAYOUT.origin_col, LAYOUT.origin_row) == (0.0, 0.0)


def test_corridor_fits_burger():
    burger_diameter = 0.21
    assert LAYOUT.cell - LAYOUT.thickness > burger_diameter + 2 * 0.1


def test_goals_are_free_cell_centres_with_stated_hops():
    hops = LAYOUT.bfs_hops()
    for g in GOALS:
        cell = LAYOUT.world_to_cell(g['x'], g['y'])
        assert cell is not None, g
        assert LAYOUT.cell_center(*cell) == (g['x'], g['y']), g
        assert hops[cell] == g['hops'], (g['name'], hops[cell])


def test_goals_far_from_walls():
    for g in GOALS:
        for w in LAYOUT.walls():
            dx = max(abs(g['x'] - w.cx) - w.length_x / 2, 0.0)
            dy = max(abs(g['y'] - w.cy) - w.length_y / 2, 0.0)
            assert (dx * dx + dy * dy) ** 0.5 >= 0.2, (g['name'], w.name)


def test_generated_world_matches_layout():
    import xml.etree.ElementTree as ET
    for name, world in (('maze.world', 'maze'), ('maze_dqn.world', 'dqn')):
        root = ET.parse(PKG / 'worlds' / name).getroot()
        assert root.find('world').get('name') == world
        links = root.findall(".//model[@name='maze']/link")
        assert len(links) == len(LAYOUT.segments)
        plugins = {p.get('name') for p in root.iter('plugin')}
        assert 'gz::sim::systems::Sensors' in plugins


def test_groundtruth_map_scores_perfectly():
    from tb3_maze.map_accuracy import evaluate
    m, _ = evaluate(LAYOUT, PKG / 'maps' / 'maze_groundtruth.yaml')
    assert m['wall_f1'] == 1.0 and m['false_wall_cells'] == 0


def test_dqn_lidar_gives_24_front_rays():
    """The official agent needs 2 + 24 inputs; check our 48-sample choice."""
    import math
    samples, max_angle = 48, 6.28
    inc = max_angle / (samples - 1)
    front = [i for i in range(samples)
             if 0 <= i * inc <= math.pi / 2 or 3 * math.pi / 2 <= i * inc <= 2 * math.pi]
    assert len(front) == 24
