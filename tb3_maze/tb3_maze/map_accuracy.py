#!/usr/bin/env python3
"""Score a saved SLAM map against the maze ground truth.

    ros2 run tb3_maze map_accuracy maps/maze_map.yaml [--tol 1] [--png overlay.png]
    python3 tb3_maze/map_accuracy.py maps/maze_map.yaml          # no ROS needed

Because the robot spawns at the world origin, the SLAM `map` frame coincides
with the world frame, so the two can be compared cell-by-cell without any
registration step.

Reported:
  wall precision   % of SLAM occupied cells within `tol` cells of a real wall
  wall recall      % of real-wall cells within `tol` cells of a SLAM occupied cell
  wall F1          harmonic mean of the two
  mean wall error  mean distance (m) from SLAM occupied cells to the nearest real wall
  free coverage    % of the maze's free floor that SLAM marked as free (explored)
  false walls      SLAM occupied cells > 2*tol cells from any real wall (speckle)
"""

import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

try:
    from tb3_maze.maze_geometry import MazeLayout, default_layout_path
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from tb3_maze.maze_geometry import MazeLayout, default_layout_path


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:                      # magic, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1                                    # single whitespace before raster
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5':
        raise ValueError(f'{path}: only binary PGM (P5) is supported, got {magic!r}')
    dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
    img = np.frombuffer(data, dtype=dtype, count=w * h, offset=pos).reshape(h, w)
    return img.astype(np.float64) / maxval


def load_map(yaml_path: Path):
    meta = yaml.safe_load(yaml_path.read_text())
    img = read_pgm(yaml_path.parent / meta['image'])
    if meta.get('negate', 0):
        img = 1.0 - img
    occ_p = 1.0 - img                            # map_server: dark = occupied
    occupied = occ_p > meta.get('occupied_thresh', 0.65)
    free = occ_p < meta.get('free_thresh', 0.25)
    # flip so row 0 is the lowest y (origin is the bottom-left pixel)
    return (np.flipud(occupied), np.flipud(free), float(meta['resolution']),
            float(meta['origin'][0]), float(meta['origin'][1]))


def dilate(mask: np.ndarray, r: int) -> np.ndarray:
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def evaluate(layout: MazeLayout, map_yaml: Path, tol: int = 1):
    slam_occ, slam_free, res, ox, oy = load_map(map_yaml)
    h, w = slam_occ.shape
    gt_occ = layout.rasterize(res, ox, oy, w, h)
    inside = layout.inside_maze(res, ox, oy, w, h)
    gt_free = inside & ~dilate(gt_occ, 1)

    near_gt = dilate(gt_occ, tol)
    near_slam = dilate(slam_occ, tol)
    n_slam, n_gt = int(slam_occ.sum()), int(gt_occ.sum())
    precision = (slam_occ & near_gt).sum() / max(n_slam, 1)
    recall = (gt_occ & near_slam).sum() / max(n_gt, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)

    # mean distance from SLAM occupied cells to the nearest GT wall cell
    sy, sx = np.nonzero(slam_occ)
    gy, gx = np.nonzero(gt_occ)
    if len(sx) and len(gx):
        d = np.sqrt((sx[:, None] - gx[None, :]) ** 2 + (sy[:, None] - gy[None, :]) ** 2)
        nearest = d.min(axis=1) * res
        mean_err = float(nearest.mean())
        false_walls = int((nearest > 2 * tol * res).sum())
    else:
        mean_err, false_walls = float('nan'), n_slam

    coverage = (slam_free & gt_free).sum() / max(gt_free.sum(), 1)
    return {
        'map': str(map_yaml), 'resolution_m': res, 'tolerance_cells': tol,
        'slam_wall_cells': n_slam, 'gt_wall_cells': n_gt,
        'wall_precision': float(precision), 'wall_recall': float(recall), 'wall_f1': float(f1),
        'mean_wall_error_m': mean_err, 'false_wall_cells': false_walls,
        'free_coverage': float(coverage),
    }, (slam_occ, slam_free, gt_occ, res, ox, oy)


def save_overlay(grids, path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    slam_occ, slam_free, gt_occ, res, ox, oy = grids
    h, w = slam_occ.shape
    rgb = np.full((h, w, 3), 0.75)
    rgb[slam_free] = (1.0, 1.0, 1.0)
    rgb[gt_occ] = (0.55, 0.75, 1.0)                 # ground truth: light blue
    rgb[slam_occ] = (0.1, 0.1, 0.1)                 # SLAM walls: black
    rgb[slam_occ & ~dilate(gt_occ, 1)] = (0.9, 0.1, 0.1)   # off-wall SLAM cells: red
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(rgb, origin='lower', extent=(ox, ox + w * res, oy, oy + h * res))
    ax.set_title('SLAM map (black) vs ground truth (blue); red = off-wall')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml', type=Path)
    ap.add_argument('--layout', type=Path, default=None)
    ap.add_argument('--tol', type=int, default=1, help='tolerance in cells (default 1)')
    ap.add_argument('--png', type=Path, default=None, help='write an overlay image')
    args = ap.parse_args(argv)

    layout = MazeLayout.load(args.layout or default_layout_path())
    m, grids = evaluate(layout, args.map_yaml, args.tol)
    print(f'\nSLAM map accuracy: {m["map"]}  (res {m["resolution_m"]} m, '
          f'tolerance {m["tolerance_cells"]} cell)')
    print(f'  wall precision   {100 * m["wall_precision"]:6.1f} %')
    print(f'  wall recall      {100 * m["wall_recall"]:6.1f} %')
    print(f'  wall F1          {100 * m["wall_f1"]:6.1f} %')
    print(f'  mean wall error  {100 * m["mean_wall_error_m"]:6.2f} cm')
    print(f'  free coverage    {100 * m["free_coverage"]:6.1f} %')
    print(f'  false wall cells {m["false_wall_cells"]:6d}\n')
    if args.png:
        save_overlay(grids, args.png)
        print(f'  overlay -> {args.png}\n')


if __name__ == '__main__':
    main()
