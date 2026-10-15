#!/usr/bin/env bash
# One-shot, idempotent environment setup for the whole team.
# Target: Ubuntu 24.04 (Noble) — the only OS ROS 2 Jazzy supports.
#
#   bash scripts/setup_env.sh            # ROS 2 Jazzy + Gazebo Harmonic + TB3 + Nav2 + SLAM
#   bash scripts/setup_env.sh --with-dqn # also turtlebot3_machine_learning + TensorFlow
#
# Safe to re-run: every step checks before it acts.
set -euo pipefail

WITH_DQN=0
[[ "${1:-}" == "--with-dqn" ]] && WITH_DQN=1

WS="$HOME/tb3_maze_ws"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
say() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- OS check
. /etc/os-release
if [[ "${VERSION_ID}" != "24.04" ]]; then
  echo "ERROR: Ubuntu 24.04 required for ROS 2 Jazzy (found ${PRETTY_NAME})." >&2
  exit 1
fi

# ------------------------------------------------------------- locale
say "Locale"
sudo apt-get update -y
sudo apt-get install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ------------------------------------------------------ ROS 2 apt source
say "ROS 2 apt repository"
sudo add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F\" '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt-get update -y

# ------------------------------------------------------- ROS 2 packages
say "ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3, Nav2, slam_toolbox"
sudo apt-get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-teleop \
  ros-jazzy-turtlebot3-navigation2 ros-jazzy-turtlebot3-cartographer \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  python3-colcon-common-extensions python3-rosdep \
  python3-yaml python3-numpy python3-matplotlib python3-pip

# turtlebot3_gazebo: apt if released, else source (jazzy branch)
mkdir -p "$WS/src"
if apt-cache show ros-jazzy-turtlebot3-gazebo >/dev/null 2>&1; then
  sudo apt-get install -y ros-jazzy-turtlebot3-gazebo
elif [[ ! -d "$WS/src/turtlebot3_simulations" ]]; then
  say "turtlebot3_simulations from source (jazzy)"
  git clone -b jazzy --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    "$WS/src/turtlebot3_simulations"
fi

if [[ $WITH_DQN -eq 1 ]]; then
  say "turtlebot3_machine_learning (jazzy) + TensorFlow (CPU)"
  if [[ ! -d "$WS/src/turtlebot3_machine_learning" ]]; then
    git clone -b jazzy --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning.git \
      "$WS/src/turtlebot3_machine_learning"
  fi
  python3 -c "import tensorflow" 2>/dev/null || \
    pip3 install --user --break-system-packages tensorflow-cpu
fi

# ------------------------------------------------------------ rosdep
say "rosdep"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update

# ---------------------------------------------------- link this repo in
if [[ "$REPO_DIR" != "$WS/src/"* ]]; then
  ln -sfn "$REPO_DIR" "$WS/src/tb3_maze_navigation"
fi

# ------------------------------------------------------------- build
say "Build workspace $WS"
# shellcheck disable=SC1091
set +u   # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# ------------------------------------------------------------ bashrc
say "~/.bashrc"
add_line() { grep -qxF "$1" "$HOME/.bashrc" || echo "$1" >> "$HOME/.bashrc"; }
add_line "source /opt/ros/jazzy/setup.bash"
add_line "source $WS/install/setup.bash"
add_line "export TURTLEBOT3_MODEL=burger"
add_line "export ROS_DOMAIN_ID=30"

say "Done. Open a NEW terminal, then:  ros2 launch tb3_maze maze.launch.py"
