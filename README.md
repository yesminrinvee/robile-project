```markdown
# Robile Project - ROS2 Autonomous Navigation

This repository contains a complete implementation of autonomous navigation and exploration for the Robile robot platform using ROS2. The project includes localization (particle filter), path planning (A* + potential field), and frontier-based exploration.

## Project Overview

This project implements a complete navigation system for the Robile robot platform. It includes:

- **Localization**: Monte Carlo particle filter (MCL) implementation
- **Path Planning**: A* global planner + Potential field local planner
- **Autonomous Exploration**: Frontier-based exploration with SLAM integration
- **Visualization**: RViz displays for particles, paths, and frontiers

## Repository Structure

```
robile_project_planner/
├── robile_project_planner/
│   ├── localization_node.py      # Particle filter (MCL)
│   ├── final_navigation.py        # A* + Potential field navigation
│   ├── exploration_node.py        # Frontier-based exploration
│   ├── planner_node.py            # Potential field planner
│   └── integrated_navigation.py   # A* + waypoint integration
├── launch/                        # Launch files
├── setup.py                       # Package configuration
└── package.xml                    # ROS2 package dependencies
```

## Prerequisites

- Ubuntu 22.04 or later
- ROS2 Humble
- Python 3.10+
- Robile robot or simulation environment

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/yesminrinvee/robile-project.git robile_project_planner
```

### 2. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select robile_project_planner
source install/setup.bash
```

## Usage

### Localization (Part 2)

The particle filter tracks robot pose using odometry and laser scan.

```bash
# Terminal 1: Robot base
ssh robile@<robot-ip>
ros2 launch robile_bringup robot.launch

# Terminal 2: Navigation stack (map)
export ROBOT_NAME=robile4
export ROS_DOMAIN_ID=4
ros2 launch robile_navigation robile_nav2_bringup.launch.py

# Terminal 3: Your localization node
export ROS_DOMAIN_ID=4
ros2 run robile_project_planner localization

# Terminal 4: RViz
rviz2
```

**In RViz:**
- Fixed Frame = `map`
- Add Map display (topic `/map`)
- Add PoseArray (topic `/particles`, color red)

### Path Planning (Part 1)

A* global planner with potential field local planner.

```bash
# After localization is running, in a new terminal:
export ROS_DOMAIN_ID=4
ros2 run robile_project_planner final_navigation
```

**In RViz:**
- Add Path display (topic `/planned_path`, color blue)
- Click "2D Goal Pose" to set goal
- Robot follows planned path and avoids obstacles

### Exploration (Part 3)

Frontier-based autonomous exploration.

```bash
# Terminal 1: Robot base
ssh robile@<robot-ip>
ros2 launch robile_bringup robot.launch

# Terminal 2: SLAM Toolbox
export ROBOT_NAME=robile4
export ROS_DOMAIN_ID=4
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Exploration node
export ROS_DOMAIN_ID=4
ros2 run robile_project_planner exploration

# Terminal 4: Navigation node
export ROS_DOMAIN_ID=4
ros2 run robile_project_planner final_navigation

# Terminal 5: RViz
rviz2
```

**In RViz:**
- Add Marker display (topic `/frontiers`, color red)
- Robot autonomously explores unknown areas

## Code Description

### localization_node.py
- Particle filter with 2000 particles
- Motion model with odometry
- Measurement model with laser scan
- Publishes particles to `/particles` topic

### final_navigation.py
- A* path planning on grid map
- Waypoint extraction and following
- Potential field obstacle avoidance
- Publishes path to `/planned_path` topic

### exploration_node.py
- Frontier detection from occupancy grid
- Sends goals to unexplored areas
- Publishes frontiers to `/frontiers` topic

## Troubleshooting

### Map Not Loading
```bash
export ROBOT_ENV=updated_map  # or map_03
ros2 launch robile_navigation robile_nav2_bringup.launch.py
```

### No Particles Appearing
- Check if initial pose is set in RViz
- Verify `/particles` topic is being published: `ros2 topic echo /particles --once`

### Robot Not Moving
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Verify odometry is being published: `ros2 topic echo /odom --once`

### Frontiers Not Showing
- Verify Marker display topic is set to `/frontiers`
- Check exploration node is running: `ros2 node list | grep exploration`
- Drive robot to create unknown areas

### RTPS_TRANSPORT_SHM Error
```bash
rm -rf /dev/shm/fastrtps_*
```

### ROS Domain ID Conflicts
```bash
export ROS_DOMAIN_ID=4  # For Robile4
echo "export ROS_DOMAIN_ID=4" >> ~/.bashrc
source ~/.bashrc
```

## Team Contributions

- **Yesmin Akter**: Mapping, SLAM integration, Particle filter implementation, A* planner, Potential field navigation, Exploration node, Integration testing, Debugging, Video recording, Testing on real robot, Report writing
- **Luc Valet**: Particle filter implementation, A* path planner Navigation stack configuration, Testing on Simulation, Report Writing

## License

This project is licensed under the MIT License.

```
