#! /bin/bash

# Set env variables (if necessary change IP address)
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=192.168.168.5

# Launch ros
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# Launch camera
rosparam set cv_camera/device_id 0
rosrun cv_camera cv_camera_node