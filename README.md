# TurtleBot3 Burger: Autonomous Maze Navigation (ROS 2 Jazzy + Gazebo Harmonic)

IE4060 Robotics and Intelligent Systems, Assignment 02 (SLIIT, 2026).

TurtleBot3 Burger navigates a custom 3 × 3 m maze (Figure 1 of the brief). The robot maps the maze with LiDAR SLAM and then reaches any goal clicked in RViz. Two planning approaches are implemented and evaluated with the same metrics:

- Classical: **Nav2**, using NavFn Dijkstra, NavFn A\* or Smac 2D as the global planner and DWB as the controller.
- Learning: **Deep Q-Learning**, using ROBOTIS `turtlebot3_dqn` trained inside this maze.

| | |
|---|---|
| Maze world | [`tb3_maze/worlds/maze.world`](tb3_maze/worlds/maze.world), generated from [`config/maze_layout.yaml`](tb3_maze/config/maze_layout.yaml) |
| Saved SLAM map | `tb3_maze/maps/maze_map.yaml` + `.pgm` |
| Ground-truth map | [`tb3_maze/maps/maze_groundtruth.yaml`](tb3_maze/maps/maze_groundtruth.yaml) |
| Results | `results/` (CSV, summary tables, plots) |

---

## 1. Install (Ubuntu 24.04)

```bash
mkdir -p ~/tb3_maze_ws/src && cd ~/tb3_maze_ws/src
git clone https://github.com/Nivakaran-S/RIS-Assignment-02.git tb3_maze_navigation
bash tb3_maze_navigation/scripts/setup_env.sh --with-dqn   # ~15-30 min first time
# open a NEW terminal afterwards
```

The script installs ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3, Nav2, slam_toolbox and (with `--with-dqn`) `turtlebot3_machine_learning` with TensorFlow. It then builds the workspace and adds the `source` lines and `TURTLEBOT3_MODEL=burger` to `~/.bashrc`. It is safe to re-run.

After any code change, rebuild:

```bash
cd ~/tb3_maze_ws && colcon build --symlink-install && source install/setup.bash
```

## 2. Launch the maze

```bash
ros2 launch tb3_maze maze.launch.py            # Gazebo GUI + robot at (0,0)
ros2 run tb3_maze check_env                    # second terminal: health check
ros2 run turtlebot3_teleop teleop_keyboard     # optional: drive around
```

`check_env` must print **ALL CHECKS PASSED**: `/clock` advancing, `/scan` at about 5 Hz with 360 beams, `/odom`, and the TF chain `odom → base_footprint → base_scan`.

Arguments: `headless:=true` (no Gazebo GUI), `x_pose:= y_pose:= yaw:=`.

## 3. SLAM and saving the map

```bash
ros2 launch tb3_maze slam.launch.py            # Gazebo + slam_toolbox + RViz
ros2 run turtlebot3_teleop teleop_keyboard     # drive SLOWLY through every corridor
```

When every corridor shows sharp black walls and no grey gaps, save the map into the package and rebuild:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/tb3_maze_ws/src/tb3_maze_navigation/tb3_maze/maps/maze_map
cd ~/tb3_maze_ws && colcon build --symlink-install
```

Score the map against the ground truth:

```bash
ros2 run tb3_maze map_accuracy ~/tb3_maze_ws/src/tb3_maze_navigation/tb3_maze/maps/maze_map.yaml \
     --png ~/tb3_maze_results/map_overlay.png
```

## 4. Nav2 click-to-goal

```bash
ros2 launch tb3_maze navigation.launch.py                  # NavFn Dijkstra (default)
ros2 launch tb3_maze navigation.launch.py planner:=astar   # NavFn A*
ros2 launch tb3_maze navigation.launch.py planner:=smac2d  # Smac 2D
```

1. The initial pose is set automatically, because the robot spawns at the map origin. Use **2D Pose Estimate** only if the scan stops lining up with the walls.
2. Click **Nav2 Goal** in RViz, click a free cell, and drag to set the heading.
3. Every clicked goal is logged to `~/tb3_maze_results/runs.csv`.

If `maps/maze_map.yaml` doesn't exist yet, the launch falls back to the ground-truth map and prints a warning.

Tuning arguments: `max_vel:=0.26`, `inflation_radius:=0.20`, `params_file:=...`.

**VIVA demo, one command:** `ros2 launch tb3_maze demo.launch.py`

## 5. Repeatable efficiency trials

With `navigation.launch.py` running:

```bash
ros2 run tb3_maze goal_runner --ros-args -p repeats:=3 -p method:=nav2_dijkstra
```

This drives start → G1, start → G2 and start → G3 (from [`config/goals.yaml`](tb3_maze/config/goals.yaml)) three times each. Each trial is logged with:

- travel time
- driven and planned path length
- path efficiency (straight-line ÷ driven)
- minimum wall clearance
- near-collisions
- total rotation

Repeat this with `planner:=astar` and `planner:=smac2d`, then build the comparison table and charts:

```bash
ros2 run tb3_maze plot_results ~/tb3_maze_results/runs.csv     # summary.md, compare.png
```

## 6. Deep Q-Learning in the maze

```bash
ros2 launch tb3_maze dqn_maze.launch.py                                  # T1: maze + goal server
ros2 run turtlebot3_dqn dqn_environment 2>&1 | tee ~/tb3_maze_results/dqn_env.log    # T2
ros2 run turtlebot3_dqn dqn_agent --ros-args -p max_training_episodes:=300 \
    -p epsilon_decay:=2500 -p verbose:=false 2>&1 | tee ~/tb3_maze_results/dqn_agent.log  # T3
ros2 run tb3_maze plot_dqn_rewards --agent-log ~/tb3_maze_results/dqn_agent.log \
    --env-log ~/tb3_maze_results/dqn_env.log                             # curves
```

Why this differs from the ROBOTIS stage worlds:

- **48-ray LiDAR.** The official agent has a fixed 26-value input: goal distance, goal angle and 24 front rays. `dqn_maze.launch.py` spawns a copy of the Burger with a 48-sample LiDAR so the input size matches. The SLAM/Nav2 robot keeps its 360 rays.
- **Maze-aware goal server.** `maze_dqn_gazebo` replaces `turtlebot3_dqn dqn_gazebo`. It only places goals at free cell centres, and uses a curriculum: goals start ≤ 2 cell-moves away and the radius grows by one every 5 successes.
- **Evaluation mode.** `dqn_maze.launch.py eval:=true` cycles the same G1–G3 goals used for Nav2 and logs them with the same metrics.

## 7. Repository layout

```
tb3_maze_navigation/
├── README.md
├── scripts/setup_env.sh            # team environment installer
├── docs/                           # report sections, VIVA notes, hand-off notes, figures
├── results/                        # committed experiment outputs
└── tb3_maze/                       # ROS 2 package (ament_python)
    ├── config/
    │   ├── maze_layout.yaml        # maze definition (single source of truth)
    │   ├── goals.yaml              # evaluation goals G1-G3
    │   ├── nav2_params_maze.yaml   # Nav2 tuned for 0.45 m corridors
    │   └── slam_toolbox_maze.yaml  # slam_toolbox tuned for a 3 m maze
    ├── launch/  maze | slam | navigation | demo | dqn_maze .launch.py
    ├── worlds/  maze.world, maze_dqn.world        (generated)
    ├── maps/    maze_groundtruth.*, maze_map.*    (generated / SLAM)
    └── tb3_maze/
        ├── maze_geometry.py        # layout -> walls, frames, BFS, rasterisation
        ├── generate_maze.py        # writes worlds + ground-truth map + top-down figure
        ├── check_env.py            # sim health check
        ├── metrics_logger.py       # per-goal efficiency metrics -> runs.csv
        ├── goal_runner.py          # repeatable Nav2 trials
        ├── maze_dqn_gazebo.py      # DQN goal server for the maze
        ├── map_accuracy.py         # SLAM map vs ground truth
        ├── plot_results.py         # results table + charts
        └── plot_dqn_rewards.py     # DQN training curves
```

To change the maze, edit `config/maze_layout.yaml`, then run `python3 tb3_maze/tb3_maze/generate_maze.py` and rebuild.

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `package 'tb3_maze' not found` | `source ~/tb3_maze_ws/install/setup.bash` |
| `/scan` silent, `check_env` fails | GPU/GL issue in a VM: `export LIBGL_ALWAYS_SOFTWARE=1` before launching. The world already has the Sensors plugin. |
| Gazebo very slow in VMware | Enable *Accelerate 3D graphics*, give the VM ≥ 8 GB RAM / 4 cores, or use `headless:=true` |
| "extrapolation into the past" TF errors | Every node must use sim time. The launch files do this; only manual `ros2 run` nodes need `--ros-args -p use_sim_time:=true` |
| Robot won't move after a Nav2 goal | Goal is inside inflated space (too close to a wall): click nearer the cell centre |
| DQN agent: `cannot reshape array ... into shape (1,26)` | You launched the stage world or stock Burger: use `dqn_maze.launch.py` |
