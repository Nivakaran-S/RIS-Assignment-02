# Report sections (Member A): Maze Design and Evaluation Methodology

Drop these into the final report. Figures: `docs/maze_topdown.png` (generated), plus a Gazebo top-down screenshot you take in the VM.

---

## 3. Maze Design

**Layout.** The environment reproduces Figure 1 of the brief: a 6 × 6 grid of 0.5 m cells, giving a 3.0 × 3.0 m maze closed by an outer wall, with 13 interior wall segments. The robot starts in the bottom-left cell. A breadth-first search over the cell graph confirms that all 36 cells are reachable from the start. The farthest cell is 14 cell-moves away, so the maze contains both short direct routes and long winding ones (Figure X).

**Construction approach.** We did not place boxes by hand in the Gazebo GUI. The maze is defined once, as a list of wall segments in grid coordinates (`config/maze_layout.yaml`), and a generator script (`generate_maze.py`) converts it into:

1. the Gazebo world `maze.world` (SDF 1.8), with one static box link per segment;
2. a second world, `maze_dqn.world`, identical but named `dqn` for the reinforcement-learning tools;
3. a ground-truth occupancy grid in `map_server` format, used later to score the SLAM map.

Because all three come from one file, they cannot drift apart. Changing the maze is a one-line edit followed by re-running the generator, and the walls are exactly where the figure puts them, rather than wherever a mouse drag left them.

**Dimensions and clearance.** Walls are 0.05 m thick and 0.30 m high. Height only needs to exceed the LiDAR plane (about 0.18 m above the floor). Thickness was chosen from the robot's geometry:

| Quantity | Value |
|---|---|
| Cell pitch | 0.50 m |
| Wall thickness | 0.05 m |
| Clear corridor width | 0.50 − 0.05 = **0.45 m** |
| Burger footprint radius | ≈ 0.105 m (width 0.178 m) |
| Clearance per side when centred | (0.45 − 0.21) / 2 ≈ **0.12 m** |

With the 0.15–0.20 m walls suggested in generic guides, the corridor would shrink to 0.30–0.35 m and the clearance to about 0.05 m per side. At that width, Nav2's inflated costmap would cover the whole corridor, and the planner would either fail or hug walls. Thin walls keep the maze faithful to the figure's 0.5 m cell pitch and still leave a safety margin.

Each wall box is extended by half a thickness at both ends, so that corners close without gaps for the LiDAR to see through.

**Simulation plumbing.**

- *System plugins.* The world loads the Physics, UserCommands, SceneBroadcaster, Sensors (ogre2) and IMU systems. Without the Sensors system, the Burger's GPU LiDAR publishes nothing.
- *Ground plane.* It is defined inline instead of being downloaded from Gazebo Fuel, so the world loads offline.
- *Spawn and bridge.* The robot is the official TurtleBot3 Burger model. It is spawned through `ros_gz_sim create`, and its topics (`/scan`, `/odom`, `/tf`, `/imu`, `/joint_states`, `/clock`, `/cmd_vel`) are bridged to ROS 2 with `ros_gz_bridge`.
- *Coordinate frames.* The world origin is placed at the centre of the start cell, and the robot spawns there facing +x. As a result the `map`, `odom` and world frames coincide at start-up. That one decision is why:
  - Nav2 needs no manual "2D Pose Estimate" step;
  - the DQN environment's odometry-based goal distance is correct;
  - the SLAM map can be compared with the ground truth cell-for-cell, without any registration.

A health-check node (`check_env`) verifies that `/clock` advances, that `/scan` delivers 360 valid beams at about 5 Hz, that `/odom` is published, and that the TF chain `odom → base_footprint → base_scan` resolves. It is run before every hand-over and before the demo.

---

## 5.x / 6.x Evaluation Methodology (Results section)

All runs, whether Nav2 or DQN, are scored by the same node (`metrics_logger`) using the same definitions, so the comparison is like-for-like.

**Goals.** Three fixed goals were chosen with clearly different route lengths (BFS cell-moves from the start):

| Goal | Cell (col,row) | Position (m) | Shortest route |
|---|---|---|---|
| G1 | (4,3) | (2.0, 1.0) | 6 moves |
| G2 | (2,2) | (1.0, 1.5) | 9 moves, walled-in centre pocket |
| G3 | (4,1) | (2.0, 2.0) | 14 moves, farthest cell |

Each goal is attempted 3 times per method. Every trial starts from the spawn pose: `goal_runner` drives back to the start untracked between trials.

**Metrics** (per trial; averages are over successful trials):

| Metric | Definition | What it shows |
|---|---|---|
| Travel time | Sim-time from goal sent to goal reached | Speed (brief: "minimise travel time") |
| Driven length | Sum of odometry displacements | Actual distance travelled |
| Planned length | Length of the first global plan (Nav2) | Planner optimality |
| Path efficiency | Straight-line distance ÷ driven length | Detour factor; < 1 is unavoidable in a maze, but higher is better |
| Min clearance | Smallest valid LiDAR range during the trial | Safety margin to walls |
| Near-collisions | Times the LiDAR minimum fell below 0.15 m | Obstacle-avoidance quality (0.15 m is also the DQN collision threshold) |
| Total rotation | Σ abs(Δheading) | Smoothness; oscillation inflates it |
| Success rate | Goals reached ÷ attempts | Reliability |

**SLAM map quality.** The saved SLAM map is scored against the generator's ground-truth grid (`map_accuracy`):

- *Wall precision:* the share of mapped wall cells within one cell (5 cm) of a real wall.
- *Wall recall:* the share of real wall cells that were mapped.
- *Mean wall error:* in centimetres.
- *Free-space coverage:* the share of the maze floor explored.

This gives the mapping section an objective number instead of a visual impression.
