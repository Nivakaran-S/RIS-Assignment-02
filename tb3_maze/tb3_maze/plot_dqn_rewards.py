#!/usr/bin/env python3
"""Training curves from the official turtlebot3_dqn logs.

Capture the logs while training:
    ros2 run turtlebot3_dqn dqn_environment 2>&1 | tee ~/tb3_maze_results/dqn_env.log
    ros2 run turtlebot3_dqn dqn_agent --ros-args ... 2>&1 | tee ~/tb3_maze_results/dqn_agent.log

Then:
    ros2 run tb3_maze plot_dqn_rewards --agent-log ~/tb3_maze_results/dqn_agent.log \
                                       --env-log ~/tb3_maze_results/dqn_env.log

Parses the agent's  "Episode: N score: S memory length: M epsilon: E"  lines and
the environment's "Goal Reached" / "Collision happened" / "Time out!" lines, and
writes dqn_training.png (score, moving average, epsilon, rolling success rate)
plus dqn_training.csv.
"""

import argparse
import csv
from pathlib import Path
import re

EP_RE = re.compile(r'Episode:\s*(\d+)\s+score:\s*([-\d.eE+]+)\s+memory length:\s*(\d+)'
                   r'\s+epsilon:\s*([-\d.eE+]+)')
OUTCOME_RE = re.compile(r'(Goal Reached|Collision happened|Time out!)')
OUTCOME = {'Goal Reached': 'success', 'Collision happened': 'collision', 'Time out!': 'timeout'}


def moving(vals, k):
    out = []
    for i in range(len(vals)):
        w = vals[max(0, i - k + 1):i + 1]
        out.append(sum(w) / len(w))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent-log', type=Path, required=True)
    ap.add_argument('--env-log', type=Path, default=None)
    ap.add_argument('--window', type=int, default=20)
    ap.add_argument('--out', type=Path, default=None)
    args = ap.parse_args(argv)
    out = args.out or args.agent_log.parent

    eps = [m.groups() for m in EP_RE.finditer(args.agent_log.read_text(errors='ignore'))]
    if not eps:
        raise SystemExit(f'No "Episode: .. score: .." lines in {args.agent_log}')
    episode = [int(e[0]) for e in eps]
    score = [float(e[1]) for e in eps]
    epsilon = [float(e[3]) for e in eps]

    outcomes = []
    if args.env_log and args.env_log.exists():
        outcomes = [OUTCOME[m] for m in
                    OUTCOME_RE.findall(args.env_log.read_text(errors='ignore'))]
    outcomes = (outcomes + [''] * len(episode))[:len(episode)]
    success = [1.0 if o == 'success' else 0.0 for o in outcomes]

    with open(out / 'dqn_training.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['episode', 'score', 'epsilon', 'outcome'])
        w.writerows(zip(episode, score, epsilon, outcomes))

    n_s = sum(1 for o in outcomes if o == 'success')
    n_c = sum(1 for o in outcomes if o == 'collision')
    n_t = sum(1 for o in outcomes if o == 'timeout')
    last = outcomes[-args.window:]
    print(f'{len(episode)} episodes | success {n_s}, collision {n_c}, timeout {n_t} | '
          f'last {len(last)}: {sum(o == "success" for o in last)} successes | '
          f'final epsilon {epsilon[-1]:.3f}')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax1 = plt.subplots(figsize=(9, 4.5))
    ax1.plot(episode, score, color='#9ecae1', linewidth=0.8, label='episode score')
    ax1.plot(episode, moving(score, args.window), color='#08519c', linewidth=2,
             label=f'{args.window}-episode mean')
    ax1.set_xlabel('episode')
    ax1.set_ylabel('score (sum of rewards)')
    ax1.grid(linewidth=0.3)
    ax2 = ax1.twinx()
    ax2.plot(episode, epsilon, color='#999999', linestyle='--', label='epsilon')
    if args.env_log:
        ax2.plot(episode, moving(success, args.window), color='#2ca02c', linewidth=2,
                 label=f'success rate ({args.window}-ep)')
    ax2.set_ylim(0, 1.05)
    ax2.set_ylabel('epsilon / success rate')
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=8)
    ax1.set_title('DQN training in the maze')
    fig.tight_layout()
    fig.savefig(out / 'dqn_training.png', dpi=150)
    print(f'wrote {out / "dqn_training.png"}, {out / "dqn_training.csv"}')


if __name__ == '__main__':
    main()
