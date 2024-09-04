# Autonomous-Turtlebot
Autonomous Turtle Bot capable of performing autonomous SLAM using Cartogrpaher and NAV2 in ROS2 Humble

## Project Overview
This project combines ROS 2 Humble, Cartographer for SLAM, Nav2 for navigation, and a custom exploration node to create an autonomous robot capable of mapping an unknown environment. The system is designed to work with a TurtleBot3 in a Gazebo simulation, but can be adapted for other robots and real-world scenarios.
Key features:

- Autonomous exploration and mapping
- Integration with Cartographer for SLAM
- Nav2 for path planning, control, and obstacle avoidance
- Custom exploration strategy for efficient area coverage
- Visualization of frontiers and robot path in RViz

## Dependencies
- Ubuntu 22.04 (or compatible)
- ROS 2 Humble
- Gazebo
- TurtleBot3 packages for ROS 2
- NAV2
- Cartographer
- Python 3.8+
- NumPy
- scikit-learn

## Installation

Set up your ROS 2 Humble workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
Clone this repository into your workspace:
```bash
git clone https://github.com/gadirajus13/autonomous_tb.git
```

## Install dependencies:

```bash
sudo apt update
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo
pip3 install numpy scikit-learn
```

Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage
1. Launch the simulation and start the autonomous exploration:

```bash
ros2 launch autonomous_tb exploration.launch.py
```
This will start Gazebo with a TurtleBot3 in a house environment, launch Cartographer for SLAM, start Nav2 for navigation, and run the custom exploration node.

2. Monitor the progress:
- Use RViz to visualize the map building process, robot's path, and detected frontiers.
- To view the frontier markers, add a Marker Array to your RViz and select the 'frontiers' topic
- Check the terminal output for status updates and any error messages.

4. To stop the exploration:
- Save the map using the typical Nav2 save methodology in the terminal
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```
- Press Ctrl+C in the terminal where you launched the exploration node to terminate the program once map is saved.


## Project Structure

- ```exploration_node.py```: Main Python script implementing the autonomous exploration algorithm.
- ```exploration.launch.py```: Launch file that sets up the entire system, including Gazebo, Cartographer, Nav2, and the exploration node.
- ```config/nav2_params.yaml```: Configuration file for Nav2 parameters.

## Configuration
You can modify the following files to adjust the behavior of the system:

- ```exploration_node.py```: Adjust exploration parameters such as cluster_tolerance, min_frontier_size, obstacle_clearance, and min_goal_distance.
- ```config/nav2_params.yaml```: Modify Nav2 parameters to fine-tune navigation behavior.
- ```exploration.launch.py```: Change the Gazebo world, robot starting position, or add/remove nodes from the launch process.

## Demo
Here is a demonstration of the exploration node running in the default Gazebo House and World environments
![Gazebo House](demo/Gazebo_house.gif)
![Gazebo House Occupancy Grid](demo/house_map.png)
![Gazebo World Occupancy Grid](demo/gazebo_world_map.png)

## Data Collection Node
If you want to collect data from the lidar and camera while running your simulation. Use the following,

To launch the Gazebo simulation and run the data collection node for the camera's and lidar from the Turtlebot Waffle Pi, run:
```
 ros2 launch autonomous_tb data_collection_launch.py
```

To control the turtlebot during data collection, in another terminal window run:
```
ros2 run turtlebot3_teleop teleop_keyboard
```
## Methodologies
### Code Structure and Functionality
The main functionality of this project is implemented in the ExplorationNode class within exploration_node.py. This node orchestrates the autonomous exploration process by integrating SLAM, navigation, and frontier detection.

#### Key components of the ExplorationNode:
- Initialization: Sets up ROS 2 subscribers, publishers, action clients, and service clients necessary for interfacing with other nodes (map, odometry, AMCL, Nav2).
- Map Processing: Receives and processes occupancy grid maps from Cartographer.
- Frontier Detection: Implements a custom frontier detection algorithm to identify unexplored areas.
- Path Planning: Utilizes Nav2 for planning and executing paths to selected frontiers.
- Exploration Loop: Continuously detects frontiers, clusters and filters frontiers based on surroundings, selects the best one, and navigates to it until the environment is fully explored.

### Frontier Detection
The frontier detection algorithm is a crucial part of the exploration process:

Detect Frontiers (detect_frontiers method):
- Iterates through the occupancy grid map.
- Identifies cells that are free (probability < occupied_threshold) and adjacent to unknown areas(-1).
- Converts grid coordinates to world coordinates.

Cluster Frontiers (cluster_frontiers method):
- Uses DBSCAN (Density-Based Spatial Clustering of Applications with Noise) to group nearby frontier points.
- Calculates the centroid of each cluster to represent a single frontier.

Filter Obstacles (filter_obstacles method):
- Ensures that selected frontiers are not too close to obstacles.
- Checks a circular area around each frontier point for occupancy.

Select Best Frontier (select_best_frontier method):
- Calculates distances from the robot to all frontier points.
- Sorts frontiers based on distance (TODO: Add other criteria (e.g., information gain potential))

### Integration with Nav2
Nav2 (Navigation2) is used for path planning and execution:

Action Client: 
- The node creates a NavigateToPose action client to send navigation goals to Nav2.

Send Goal (send_goal method):
- Constructs a NavigateToPose.Goal message with the selected frontier's coordinates and sends the goal to Nav2 asynchronously.

Feedback and Result Handling:
- ```goal_response_callback```: Handles the initial response from Nav2 (goal accepted or rejected).
- ```get_result_callback```: Processes the final result of the navigation action.
- ```feedback_callback```: Receives and processes periodic feedback during navigation (e.g. navigation cancelled).

Path Validation (is_path_free_nav2 method):
- Uses Nav2's IsPathValid service to check if a path to a frontier is obstacle-free before attempting navigation.

### Use of Cartographer
Cartographer is used for Simultaneous Localization and Mapping (SLAM):

- Integration: Cartographer is launched as a separate node via the exploration.launch.py file.
- Map Subscription: The exploration node subscribes to the occupancy grid map topic published by Cartographer (/map).
- Map Updates: As Cartographer updates the map, the exploration node receives these updates and uses them for frontier detection and navigation planning.
- Localization: Cartographer provides real-time localization of the robot within the map using odometry data, which is crucial for accurate navigation and frontier selection.

### Exploration Process
The overall exploration process follows these steps:

1. Initialize the system (Gazebo, Cartographer, Nav2, Exploration Node).
2. Wait for the initial map and robot pose.
3. Detect frontiers in the current map.
4. Select the best frontier based on distance and other criteria.
5. Use Nav2 to plan and execute a path to the selected frontier.
6. During navigation, continuously update the map and re-evaluate frontiers.
7. Once the goal is reached or becomes unreachable, select a new frontier.
8. Repeat steps 3-7 until no more frontiers are detected or a termination condition is met.

This autonomous exploration system combines the mapping capabilities of Cartographer, the navigation and planning functions of Nav2, and a custom frontier-based exploration strategy to efficiently explore and map unknown environments.
