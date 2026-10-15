#!/usr/bin/env python3
"""Summarise runs.csv into the report's results table and charts.

    ros2 run tb3_maze plot_results [~/tb3_maze_results/runs.csv] [--out ~/tb3_maze_results]

Writes:
  summary.md   markdown table: method x goal -> success rate, mean +/- std of
               time, driven length, efficiency, min clearance, near-collisions
  summary.csv  same numbers, machine-readable
  compare.png  grouped bar charts (time, path length, efficiency) per goal
"""

import argparse
from collections import defaultdict
import csv
from pathlib import Path
import re
import statistics as st

METRICS = [('time_s', 'Time (s)'), ('driven_m', 'Driven (m)'), ('planned_m', 'Planned (m)'),
           ('efficiency', 'Efficiency'), ('min_clear_m', 'Min clear (m)'),
           ('near_collisions', 'Near-coll.'), ('rotation_rad', 'Rotation (rad)')]


def goal_of(label: str) -> str:
    """'G1_r2' -> 'G1'; anything else is kept as-is."""
    m = re.match(r'(G\d+)', label)
    return m.group(1) if m else label


def fmt(vals):
    if not vals:
        return '-'
    if len(vals) == 1:
        return f'{vals[0]:.2f}'
    return f'{st.mean(vals):.2f} ± {st.stdev(vals):.2f}'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', type=Path,
                    default=Path.home() / 'tb3_maze_results' / 'runs.csv')
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('--all', action='store_true',
                    help='include click/curriculum episodes, not just G1..Gn trials')
    args = ap.parse_args(argv)
    out = args.out or args.csv.parent
    out.mkdir(parents=True, exist_ok=True)

    with open(args.csv, newline='') as f:
        rows = list(csv.DictReader(f))
    if not args.all:
        rows = [r for r in rows if re.match(r'G\d+', r['label'])]
    if not rows:
        raise SystemExit(f'No G1..Gn trial rows in {args.csv} (use --all for clicks).')

    groups = defaultdict(list)
    for r in rows:
        groups[(goal_of(r['label']), r['method'])].append(r)

    lines = ['| Goal | Method | Success | ' + ' | '.join(n for _, n in METRICS) + ' |',
             '|' + '---|' * (3 + len(METRICS))]
    table = []
    for (goal, method) in sorted(groups):
        rs = groups[(goal, method)]
        ok = [r for r in rs if r['result'] == 'succeeded']
        rec = {'goal': goal, 'method': method, 'n': len(rs), 'success': len(ok)}
        cells = []
        for key, _ in METRICS:
            vals = [float(r[key]) for r in ok if r[key] not in ('', None)]
            rec[key + '_mean'] = st.mean(vals) if vals else ''
            rec[key + '_std'] = st.stdev(vals) if len(vals) > 1 else ''
            cells.append(fmt(vals))
        table.append(rec)
        lines.append(f'| {goal} | {method} | {len(ok)}/{len(rs)} | ' + ' | '.join(cells) + ' |')

    md = ('Metrics are over successful trials only (mean ± std).\n'
          'Efficiency = straight-line distance / driven distance. Near-coll. = times the LiDAR '
          'minimum fell below 0.15 m.\n\n' + '\n'.join(lines) + '\n')
    (out / 'summary.md').write_text(md, encoding='utf-8')
    with open(out / 'summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print(md)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed: skipping compare.png')
        return
    goals = sorted({t['goal'] for t in table})
    methods = sorted({t['method'] for t in table})
    lookup = {(t['goal'], t['method']): t for t in table}
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    width = 0.8 / max(len(methods), 1)
    for ax, (key, title) in zip(axes, [('time_s', 'Travel time (s)'),
                                       ('driven_m', 'Driven path length (m)'),
                                       ('efficiency', 'Path efficiency')]):
        for i, m in enumerate(methods):
            means, errs = [], []
            for g in goals:
                t = lookup.get((g, m), {})
                means.append(t.get(key + '_mean') or 0.0)
                errs.append(t.get(key + '_std') or 0.0)
            xs = [j + (i - (len(methods) - 1) / 2) * width for j in range(len(goals))]
            ax.bar(xs, means, width, yerr=errs, capsize=3, label=m)
        ax.set_xticks(range(len(goals)))
        ax.set_xticklabels(goals)
        ax.set_title(title)
        ax.grid(axis='y', linewidth=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'compare.png', dpi=150)
    print(f'wrote {out / "summary.md"}, {out / "summary.csv"}, {out / "compare.png"}')


if __name__ == '__main__':
    main()
