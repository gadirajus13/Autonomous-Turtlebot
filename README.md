# Autonomous-Turtlebot
Autonomous Turtle Bot capable of obstacle detection, SLAM, navigation, and motion planning using ROS2 and Gazebo

To use the following code, please download all the files and place them in your ROS2 Humble workspacce under the package name
`autonomous_tb` and build the ROS2 package in your workspace.

To launch the Gazebo simulation and run the data collection node for the camera's and lidar from the Turtlebot Waffle Pi, run:
```
 ros2 launch autonomous_tb data_collection_launch.py
```

To control the turtlebot during data collection, in another terminal window run:
```
ros2 run turtlebot3_teleop teleop_keyboard
```
