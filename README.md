# AST_assignments

This project demonstrates a multi-robot Gazebo simulation setup using ROS 2. It spawns multiple robots in a Gazebo environment and optionally performs laser scanning for one of the robots.

Prerequisites

    ROS 2 
    Gazebo installed
    Python 3


Installation

1. Clone this repository to your ROS 2 workspace:

git clone <repository_url>

2. Build your ROS 2 workspace:

colcon build


Running the Simulation

1. Open a terminal and source your ROS 2 workspace:
   source /path/to/your/ros2_ws/install/setup.bash

2. Launch the Gazebo simulation:
  ros2 launch <package_name> <launch_file>.py
  eg: ros2 launch robile safety robile safety.py

Customization

Adjust the number of robots in the generate_launch_description() function in your launch file as needed.
Eg: Change the variable number_of_robots = 2, for spawning two robots


Notes

Ensure that your URDF files and Gazebo world files are correctly configured and located in the appropriate directories.
