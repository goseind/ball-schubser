#! /bin/bash
#roslaunch usb_cam usb_cam-test.launch
rosparam set cv_camera/device_id 0
rosrun cv_camera cv_camera_node