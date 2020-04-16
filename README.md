# LeapMotion-Ros-Telemanipulation
Telemanipulation using Leap Motion devices on a ROS Panda robot.

## Requirements:

- [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)

- [Leap Motion SDK](https://developer.leapmotion.com/setup/desktop)

- [Leap Motion ROS Package](https://github.com/ros-drivers/leap_motion)

- [MoveIt!](https://moveit.ros.org/install/)

## Setup:

1. **Terminal** -> sudo Leapd

2. **Terminal** -> roscore

3. **Terminal** -> roslaunch leap_motion demo.launch

4. **Terminal** -> roslaunch panda_moveit_config demo.launch

5. **Terminal** -> rosrun moveit_tutorials pyMoveGroup.py

(Optional) **Terminal** -> LeapControlPanel
