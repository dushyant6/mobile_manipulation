#!/bin/bash

# Change directory to urdf folder
cd src/skywalker/urdf/

# Generate URDF file from Xacro
ros2 run xacro xacro mobile_manipulator.urdf.xacro > mobile_manipulator.urdf

# Convert URDF to SDF
gz sdf -p mobile_manipulator.urdf > mobile_manipulator.sdf

# Move back to the project root directory
cd ../../..

# Build the project
colcon build

# Source the setup file
source install/local_setup.bash

# Launch the robot
ros2 launch skywalker robot.launch.py
