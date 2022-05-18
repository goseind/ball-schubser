# Ball Schubser

A robot that pushes a ball.

## Some ROS Commands

- ui: `rqt`
- list all packages: `rospack list-names`
- build packages inside `~/catkin_ws`: catkin_make
- start master: `roscore`

## Turtlebot Configuration

```bash
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=192.168.168.5
roslaunch turtlebot3_bringup turtlebot3_core.launch # bring-up cmd without LIDAR sensor
```

Manual start for webcam:

```bash
ssh ubuntu@192.168.168.4
roslaunch 
roslaunch usb_cam usb_cam-test.launch
roslaunch turtlebot3_bringup turtlebot3_core.launch # or turtlebot3_robot.launch
```

## Network Configuration

Turtlebot and PC have a static IP configured:

* Turtlebot IP: `192.168.168.4`
* Remote PC IP: `192.168.168.5` (master)

They are connected to the lab router with SSID: `Netgear`.

Refer to [Network Manager YML](turtlebot/50-cloud-init.yaml).

## Detection Node

Download weights for yolov4 https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights  
Move the weights into detect/yolo_node/weights.

Dependencies:

* tensorflow
* pandas
* opencv-python
* matplotlib

## Problems (so far..)

* The default user `ubuntu` on the Turtlebot image does not have the proper `tty` permissions. The problem could be solved by adding `ubuntu` to `root` user group, by executing: `sudo usermod -aG root ubuntu`, as the normal group used for that called `dialout` was not set for `tty`.
