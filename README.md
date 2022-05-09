# amr
Autonome mobile Roboter HS Mannheim

## ROS commands

ui: `rqt`
list all packages: `rospack list-names`
build packages inside `~/catkin_ws`: catkin_make
start master: `roscore`

## Turtlebot

```bash
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=127.0.0.1 # when using ssh port forwarding
roslaunch turtlebot3_bringup turtlebot3_robot.launch # bring-up cmd
```
