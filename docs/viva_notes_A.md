# VIVA notes: environment, integration and evaluation (Member A)

Short answers for the questions most likely to come up, with the details to back each one.

**Why generate the world from YAML instead of the Gazebo GUI?**
The YAML is the single source of truth. The world file, the DQN world and the ground-truth map are all generated from it, so they always agree. Walls land exactly on the 0.5 m grid of Figure 1, and changing the maze is a one-line edit.

**What is SDF? What is inside your world?**
SDF (Simulation Description Format) is an XML format that describes a world.
- Ours contains five system plugins: Physics, UserCommands, SceneBroadcaster, Sensors (ogre2 renderer) and IMU.
- It also has a sun light, an inline ground plane, and one static model `maze` made of 17 box links (4 outer walls and 13 interior walls). Each link has the same box for its collision and visual geometry.

**Why does the Sensors plugin matter?**
In Gazebo Harmonic, sensors are processed by a world-level system. The Burger's `gpu_lidar` is rendered by the Sensors system, so without that plugin `/scan` never publishes.

**What does ros_gz_bridge do?**
Gazebo and ROS 2 use different middleware (gz-transport vs DDS) and different message types. The bridge translates topic by topic, for example `gz.msgs.LaserScan` to `sensor_msgs/LaserScan`. Directions:
- GZ → ROS: `/scan`, `/odom`, `/tf`, `/clock`, `/imu`, `/joint_states`.
- ROS → GZ: `/cmd_vel`. In Jazzy this is a `TwistStamped`.

**Explain the TF tree.**
`map → odom → base_footprint → base_link → base_scan`.
- `map → odom` comes from SLAM during mapping, or from AMCL during navigation.
- `odom → base_footprint` comes from the DiffDrive plugin's wheel odometry.
- The fixed links below `base_footprint` come from `robot_state_publisher` using the URDF.

**Why use_sim_time?**
Gazebo publishes its own clock on `/clock`, and it can run slower or faster than real time. If one node uses wall time, its TF stamps won't match the sensor stamps. The symptom is "extrapolation into the past" errors and a smeared map. Every launch file here sets `use_sim_time` to true.

**Why spawn at (0,0) in the start-cell centre?**
It makes the `map`, `odom` and world frames identical at start-up. That gives four things for free:
- AMCL gets its initial pose automatically.
- Goal coordinates are plain maze metres.
- The DQN environment's odometry-based goal distance is correct, since it assumes odom equals world.
- The SLAM map can be scored against the ground truth with no alignment step.

**Why 0.05 m walls when guides say 0.15–0.20 m?**
With a 0.5 m pitch, 0.05 m walls leave a 0.45 m corridor. The Burger's footprint diameter is about 0.21 m, so a centred robot has about 0.12 m clearance per side. With 0.20 m walls the corridor is only 0.30 m, about 0.045 m per side. At that width the inflated costmap fills the whole corridor and Nav2 cannot plan.

**How did you size the Nav2 inflation?**
A centred robot is 0.225 m from each wall.
- The inflation radius is 0.22 m with a steep cost decay (cost_scaling_factor 10). That leaves a low-cost valley along the corridor centre-line.
- The stock 0.5 m radius paints every corridor high-cost. That makes the planner indifferent between routes and the controller oscillate.

**How do you measure efficiency?**
`metrics_logger` scores every goal the same way for Nav2 and DQN:
- *Travel time:* sim-time from goal sent to goal reached.
- *Driven length:* the sum of odometry displacements.
- *Planned length:* taken from `/plan`.
- *Path efficiency:* straight-line distance ÷ driven length.
- *Minimum LiDAR clearance:* the closest approach to a wall.
- *Near-collisions:* LiDAR readings below 0.15 m, which is the same threshold the DQN environment calls a collision.
- *Total rotation:* a smoothness measure.

**How do you know the SLAM map is good?**
`map_accuracy` overlays the saved map on the generator's ground truth. It reports wall precision and recall within 1 cell (5 cm), mean wall error in cm, and free-space coverage. We tested it: the ground truth scores 100%, and a 5 cm shifted, noisy copy drops to 73% strict precision with 1.6 cm mean error.

**Why did DQN need a different robot model?**
The official agent's network input is fixed at 26 values: goal distance, goal angle and 24 front LiDAR rays. Those front rays are the ones within ±90° of heading. With 48 LiDAR samples there are exactly 24 front rays. The stock 360-sample LiDAR gives about 181, which crashes the agent. So the DQN launch spawns a 48-ray copy of the Burger, and the Nav2/SLAM robot is untouched.

**Why did you write maze_dqn_gazebo instead of using the official dqn_gazebo?**
The official node places goals randomly in a 4.2 × 4.2 m square, which here would often fall inside walls or outside the maze. Ours samples only free cell centres. It also adds a curriculum: goals start nearby and get farther as the agent succeeds. It also has an evaluation mode that uses the same G1–G3 goals as Nav2.

**What would you change with two more weeks?**
- Give the DQN a local occupancy patch, or the next Nav2 waypoint, as its target instead of the final goal. That would be a hybrid planner where Nav2 handles global planning and DQN local control.
- Randomise the maze layout from the YAML generator for domain randomisation.
- Add a Gazebo-native contact sensor so collisions are counted exactly rather than by a LiDAR threshold.
