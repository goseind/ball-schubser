#! /bin/bash

# set ros master ip
export TURTLEBOT3_MODEL="burger"
export ROS_MASTER_URI=$(MASTER_IP)

# Launch ros
roslaunch turtlebot3_bringup turtlebot3_core.launch

# Launch camera
rosparam set cv_camera/device_id 0
rosrun cv_camera cv_camera_node