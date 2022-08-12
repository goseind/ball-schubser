#! /bin/bash
source ./config.sh

rosparam set cv_camera/device_id 0
rosrun cv_camera cv_camera_node
