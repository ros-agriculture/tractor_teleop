# tractor_teleop

A simple robot drive system for car-like vehicles.
Button control of relay service.

##
![simple_drive](http://wiki.ros.org/simple_drive?action=AttachFile&do=get&target=Simple_Drive_Diagram.png)

## Quick Start

```
TODO:  Diagram for lawn tractor
```

1. Install:
```
catkin_ws/src$ git clone https://github.com/ros-agriculture/tractor_teleop.git
catkin_ws$ catkin build
catkin_ws$ source devel/setup.bash
```
2. Launch ROS nodes:

```
$ roslaunch tractor_teleop drive_teleop.launch
$ roslaunch tractor_teleop cmd_vel_mux.launch
```

OR all-in-one launch:
```
$ roslaunch tractor_teleop drive.launch
```
3. Instructions
![](https://raw.githubusercontent.com/ros-agriculture/tractor_teleop/master/docs/teleop_params.gif)

4. Drive your robot around.

## Credits
The package uses Daniel Snider's simple_drive http://wiki.ros.org/simple_drive


