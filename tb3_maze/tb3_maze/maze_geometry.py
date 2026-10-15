"""Maze geometry shared by the world generator, ground-truth map, DQN goal
sampler and SLAM accuracy checker.

Pure Python + PyYAML + (optionally) numpy, so it runs without ROS.
"""

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


@dataclass
class Wall:
    """Axis-aligned wall box in world coordinates (metres)."""

    name: str
    cx: float
    cy: float
    length_x: float
    length_y: float


class MazeLayout:

    def __init__(self, data: dict):
        self.cell = float(data['cell_size'])
        self.cols = int(data['cols'])
        self.rows = int(data['rows'])
        self.thickness = float(data['wall_thickness'])
        self.height = float(data['wall_height'])
        self.origin_col, self.origin_row = (int(v) for v in data['origin_cell'])
        self.segments: List[Tuple[int, int, int, int]] = []
        for seg in data['walls']:
            x1, y1, x2, y2 = (int(v) for v in seg)
            if x1 != x2 and y1 != y2:
                raise ValueError(f'Wall {seg} is not axis-aligned')
            if not (0 <= min(x1, x2) and max(x1, x2) <= self.cols
                    and 0 <= min(y1, y2) and max(y1, y2) <= self.rows):
                raise ValueError(f'Wall {seg} lies outside the {self.cols}x{self.rows} grid')
            # normalise so (x1,y1) <= (x2,y2)
            self.segments.append((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))

    @classmethod
    def load(cls, path) -> 'MazeLayout':
        with open(path, 'r') as f:
            return cls(yaml.safe_load(f))

    # ------------------------------------------------------------------ frames
    def grid_to_world(self, gx: float, gy: float) -> Tuple[float, float]:
        """Grid-line coordinates (figure, y down) -> world metres (y up)."""
        wx = (gx - (self.origin_col + 0.5)) * self.cell
        wy = ((self.origin_row + 0.5) - gy) * self.cell
        return wx, wy

    def cell_center(self, col: int, row: int) -> Tuple[float, float]:
        return self.grid_to_world(col + 0.5, row + 0.5)

    def world_to_cell(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        gx = wx / self.cell + self.origin_col + 0.5
        gy = (self.origin_row + 0.5) - wy / self.cell
        col, row = int(gx // 1), int(gy // 1)
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return col, row
        return None

    def bounds(self) -> Tuple[float, float, float, float]:
        """(xmin, ymin, xmax, ymax) of the maze's outer wall centre-lines."""
        x0, y0 = self.grid_to_world(0, self.rows)
        x1, y1 = self.grid_to_world(self.cols, 0)
        return x0, y0, x1, y1

    # ------------------------------------------------------------------- walls
    def walls(self) -> List[Wall]:
        """Wall boxes. Each segment is extended by half a thickness at both
        ends so that corners close without gaps."""
        out = []
        t = self.thickness
        for i, (x1, y1, x2, y2) in enumerate(self.segments):
            ax, ay = self.grid_to_world(x1, y1)
            bx, by = self.grid_to_world(x2, y2)
            cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0
            if y1 == y2:   # horizontal in the figure -> along world x
                lx, ly = abs(bx - ax) + t, t
            else:          # vertical in the figure -> along world y
                lx, ly = t, abs(by - ay) + t
            out.append(Wall(f'wall_{i:02d}', cx, cy, lx, ly))
        return out

    # ------------------------------------------------------- cell connectivity
    def _blocked(self) -> set:
        """Set of unit edges that are walls, as ('h', gx, gy) / ('v', gx, gy).

        ('h', gx, gy): the unit edge from (gx,gy) to (gx+1,gy)
        ('v', gx, gy): the unit edge from (gx,gy) to (gx,gy+1)
        """
        blocked = set()
        for x1, y1, x2, y2 in self.segments:
            if y1 == y2:
                for gx in range(x1, x2):
                    blocked.add(('h', gx, y1))
            else:
                for gy in range(y1, y2):
                    blocked.add(('v', x1, gy))
        return blocked

    def neighbours(self, col: int, row: int, blocked=None) -> List[Tuple[int, int]]:
        blocked = self._blocked() if blocked is None else blocked
        out = []
        if col + 1 < self.cols and ('v', col + 1, row) not in blocked:
            out.append((col + 1, row))
        if col - 1 >= 0 and ('v', col, row) not in blocked:
            out.append((col - 1, row))
        if row + 1 < self.rows and ('h', col, row + 1) not in blocked:
            out.append((col, row + 1))
        if row - 1 >= 0 and ('h', col, row) not in blocked:
            out.append((col, row - 1))
        return out

    def bfs_hops(self, start: Optional[Tuple[int, int]] = None) -> Dict[Tuple[int, int], int]:
        """Cell -> number of cell-to-cell moves from `start` (default: origin cell)."""
        start = start or (self.origin_col, self.origin_row)
        blocked = self._blocked()
        dist = {start: 0}
        q = deque([start])
        while q:
            c = q.popleft()
            for n in self.neighbours(*c, blocked=blocked):
                if n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return dist

    def unreachable_cells(self) -> List[Tuple[int, int]]:
        reach = self.bfs_hops()
        return [(c, r) for r in range(self.rows) for c in range(self.cols) if (c, r) not in reach]

    # -------------------------------------------------------- rasterisation
    def rasterize(self, resolution: float, origin_x: float, origin_y: float,
                  width: int, height: int):
        """Boolean occupancy grid (numpy, row 0 = lowest y, like a ROS map)
        with True where a wall box covers the cell centre."""
        import numpy as np
        xs = origin_x + (np.arange(width) + 0.5) * resolution
        ys = origin_y + (np.arange(height) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        occ = np.zeros((height, width), dtype=bool)
        # inflate by half a cell so 5 cm walls always hit at least one cell
        pad = resolution / 2.0
        for w in self.walls():
            occ |= ((np.abs(gx - w.cx) <= w.length_x / 2.0 + pad - 1e-9)
                    & (np.abs(gy - w.cy) <= w.length_y / 2.0 + pad - 1e-9))
        return occ

    def inside_maze(self, resolution: float, origin_x: float, origin_y: float,
                    width: int, height: int):
        """Boolean grid: True for cells strictly inside the outer wall."""
        import numpy as np
        x0, y0, x1, y1 = self.bounds()
        xs = origin_x + (np.arange(width) + 0.5) * resolution
        ys = origin_y + (np.arange(height) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        return (gx > x0) & (gx < x1) & (gy > y0) & (gy < y1)


def default_layout_path() -> Path:
    """config/maze_layout.yaml from the source tree, else the installed share."""
    here = Path(__file__).resolve().parent.parent / 'config' / 'maze_layout.yaml'
    if here.exists():
        return here
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('tb3_maze')) / 'config' / 'maze_layout.yaml'
