#  Frontier Explorer — ROS 2 Autonomous Mapping

A ROS 2 Humble package that lets a TurtleBot3 explore an unknown environment completely on its own. The robot builds a map in real time using SLAM, detects unexplored regions (called *frontiers*), and navigates to them one by one until the entire area is mapped — no human input needed.

---

## How It Works

The frontier explorer node subscribes to the `/map` topic published by SLAM Toolbox. On every timer tick (every 5 seconds), it scans the occupancy grid for frontier cells — cells that are known free but sit right next to unknown space. It clusters those cells, picks the largest cluster, and sends a navigation goal to Nav2. If a goal fails or times out (60s), it gets blacklisted so the robot doesn't keep retrying the same dead end. Orange spheres visualized in RViz2 show where the current frontiers are.

---

## Prerequisites

Make sure you have these installed before anything else:

- **Ubuntu 22.04**
- **ROS 2 Humble** — [install guide](https://docs.ros.org/en/humble/Installation.html)
- **Nav2** — `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- **SLAM Toolbox** — `sudo apt install ros-humble-slam-toolbox`
- **TurtleBot3 packages** — `sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations`
- **Gazebo** (comes with turtlebot3-simulations)

Set your TurtleBot3 model in your `.bashrc` (or run it every session):

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Setup & Installation

### 1. Clone or copy the workspace

If you're cloning from GitHub:

```bash
git clone https://github.com/YOUR_USERNAME/frontier_ws.git
cd frontier_ws
```

### 2. Install dependencies

```bash
cd ~/frontier_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the workspace

```bash
cd ~/frontier_ws
colcon build
```

### 4. Source the workspace

You'll need to do this every time you open a new terminal, or add it to your `.bashrc`:

```bash
source ~/frontier_ws/install/setup.bash
```

---

## Running the Explorer

### Option A — Full launch (recommended)

This single command starts everything: Gazebo simulation, SLAM Toolbox, Nav2, the frontier explorer node, and RViz2.

```bash
ros2 launch frontier_explorer exploration.launch.py
```

The frontier node starts automatically after a 10-second delay to give Nav2 time to initialize. You'll see orange spheres appearing in RViz2 marking the frontiers, and the robot will start moving on its own.

### Option B — Nav2 only (no simulation)

If you're running on a real robot or handling the simulation separately:

```bash
ros2 launch frontier_explorer nav2_slam.launch.py
```

Then run the frontier node separately:

```bash
ros2 run frontier_explorer frontier_node
```

---

## Tunable Parameters

You can adjust the minimum frontier cluster size (smaller = more aggressive exploration, larger = only goes for big unexplored areas):

```bash
ros2 run frontier_explorer frontier_node --ros-args -p min_frontier_size:=15
```

Default is `10` cells.

---

## Visualizing in RViz2

Once everything is running, open RViz2 and add these displays:

- **Map** → Topic: `/map`
- **MarkerArray** → Topic: `/frontier_markers` (shows frontier targets as orange spheres)
- **RobotModel** (optional, for TurtleBot3 visualization)

---

## Project Structure

```
frontier_ws/
└── src/
    └── frontier_explorer/
        ├── frontier_explorer/
        │   └── frontier_node.py      # Main exploration logic
        ├── launch/
        │   ├── exploration.launch.py # Full simulation launch
        │   └── nav2_slam.launch.py   # Nav2-only launch
        ├── config/
        │   └── nav2_params.yaml      # Nav2 configuration
        ├── package.xml
        └── setup.py
```

---



---

## License

MIT
