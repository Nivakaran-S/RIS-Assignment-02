# Team hand-off: what's ready and what you do next

Everything below lives in the `tb3_maze` package. Build it once with `scripts/setup_env.sh`. After that, use `colcon build --symlink-install` when something changes.

## Everyone
- Environment: run `bash scripts/setup_env.sh --with-dqn` on Ubuntu 24.04, then open a new terminal.
- Sanity check: run `ros2 launch tb3_maze maze.launch.py`, then `ros2 run tb3_maze check_env`. It must say ALL CHECKS PASSED.
- The robot always spawns at **(0, 0) facing +x**. At start-up `map == odom == world`, so goal coordinates in any file are plain maze coordinates in metres.

## B (SLAM)
Ready for you:
- `ros2 launch tb3_maze slam.launch.py` starts Gazebo, slam_toolbox (with `config/slam_toolbox_maze.yaml`) and RViz together.

Your tasks:
1. Drive slowly with teleop: `ros2 run turtlebot3_teleop teleop_keyboard`, keeping speed under about 0.1 m/s. Visit every cell.
2. Save the map to `tb3_maze/maps/maze_map` (exact command in the README §3), then commit `maze_map.yaml` and `maze_map.pgm`.
3. Score it with `ros2 run tb3_maze map_accuracy <maze_map.yaml> --png map_overlay.png`. Put the numbers and the overlay image in the report.
4. Tune `config/slam_toolbox_maze.yaml` if walls come out doubled or blurry (check `minimum_travel_distance` and `loop_search_*`), and record what you changed and why.

## C (Nav2)
Ready for you:
- `ros2 launch tb3_maze navigation.launch.py planner:=dijkstra|astar|smac2d` brings up Nav2, RViz and the metrics logger together.
- Until B's map exists, the launch automatically uses the ground-truth map, so you can start tuning now.
- `config/nav2_params_maze.yaml` is a baseline already sized for 0.45 m corridors. It sets the inflation radius to 0.22 m, the robot radius to 0.105 m, the controller to 20 Hz, and the goal tolerance to 0.12 m. Every change from the TurtleBot3 defaults is marked `[maze]` with the reason.

Your tasks:
1. Click-to-goal demo: use the "Nav2 Goal" button in RViz.
2. Run the trials with `ros2 run tb3_maze goal_runner --ros-args -p repeats:=3 -p method:=nav2_<planner>`, once for each planner.
3. Try `max_vel:=0.26` and `inflation_radius:=0.18/0.25` as launch arguments, and keep the best combination.
4. Build the table and charts with `ros2 run tb3_maze plot_results ~/tb3_maze_results/runs.csv`. This produces `summary.md` and `compare.png`.

## D (DQN and documentation)
Ready for you:
- `ros2 launch tb3_maze dqn_maze.launch.py` starts the maze world plus `maze_dqn_gazebo`, which replaces the official `dqn_gazebo`. After it is up, run the **official** `dqn_environment` and `dqn_agent`.
- The robot in this mode has a 48-ray LiDAR, because the official agent's input is fixed at 26 values (2 goal values plus 24 front rays). Don't launch it on the stock 360-ray Burger.
- Goals use a curriculum: they start at most 2 cell-moves from the robot, and the range grows by 1 every 5 successes. Adjust with `start_hops:=` and `promote_after:=`.
- `eval:=true` uses the fixed goals G1–G3 and logs the episodes into the same `runs.csv` as Nav2, under the method name `dqn`.
- Training curves come from `ros2 run tb3_maze plot_dqn_rewards --agent-log ... --env-log ...`. The `tee` commands that capture those logs are in README §6.

Expect DQN to be far worse than Nav2 in a maze. The reward uses straight-line goal distance and angle, so walls between the robot and the goal look like dead ends to it. That is a valid, honest finding for the report (sample efficiency, no global map, reward shaping), not a failure.

## Verify in VM (not exercisable on the authoring machine)
These were written against the ROBOTIS jazzy sources but have not yet been run end-to-end:
- [ ] `maze.launch.py`: Gazebo shows the maze, the robot sits in the bottom-left cell, and `check_env` passes.
- [ ] `slam.launch.py`: a map builds as you drive.
- [ ] `navigation.launch.py` with each of the 3 planners: it reaches G1–G3.
- [ ] `goal_runner` and `metrics_logger` write rows to `runs.csv`.
- [ ] `dqn_maze.launch.py` with the official environment and agent: episodes run and the goal box appears in free cells.
