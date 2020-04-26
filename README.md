# LeapMotion-Ros-Telemanipulation
Telemanipulation using Leap Motion devices on a ROS Panda robot.

## Requirements:
-  1x Leap Motion Controller

- [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)

- [Leap Motion SDK](https://developer.leapmotion.com/setup/desktop)

- [Leap Motion ROS Package](https://github.com/ros-drivers/leap_motion)

- [MoveIt!](https://moveit.ros.org/install/)

## Setup:

Install The required software.

Follow ROS tutorials on setting up a catkin workspace.

Install Leap Motion SDK and MoveIt! package into catkin workspace and build with 'catkin make'.

Download the application script 'pyMoveGroup.py'.

Navigate to the files location and make it executable with 'sudo chmod +x'

1. **Terminal** -> sudo Leapd

2. **Terminal** -> roscore

3. **Terminal** -> roslaunch leap_motion demo.launch

4. **Terminal** -> roslaunch panda_moveit_config demo.launch

5. **Terminal** -> rosrun moveit_tutorials pyMoveGroup.py

(Optional) **Terminal** -> LeapControlPanel

Edit dpRound value to change hand movement threshold.

```self.dpRound = 1 #3 Decimal Points for Rounding```
