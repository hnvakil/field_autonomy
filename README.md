# field_autonomy
An autonomous trail-following robot. Final project for A Computational Introduction to Robotics, Fall 2022.

# Project Goal
We set out to retrofit an existing remote controlled vehicle into an autonomous outdoor navigation system using an Arduino, Raspberry Pi, smartphone running an AR app, and ROS2. Our hope was to design a system which enabled our vehicle to autonomously navigate forest trails using machine learning.

# Documentation
Primary documentation of this project can be found on [our website](https://hnvakil.github.io/field_autonomy/index.html).

# Code
* ARKitROS2Bridge
  * iOS app to upload phone pose, camera stream, and GPS location from ARKit wirelessly
* arduino
  * Arduino scripts to read commands over Serial and output PWM signals to motors
  * Contains arduino_ros ROS2 package with TreadWriter node, which communicates with Arduino over Serial
* arkit_data_streamer
  * ROS2 package to receive data from ARKitROS2Bridge app
  * Package contains separate README for install and run instructions
* trail_finding
  * ROS2 package to run trail identification ML model
* vel_calculator
  * ROS2 package containing node to command linear and angular velocity based on results of trail finding algorithm
* direction_interfaces
  * Contains custom ROS2 message type Direction
* gps_interfaces
  * Contains custom ROS2 message CoordinateStamped

# TODOs
* Complete install/run instructions for overall project
* Complete requirements.txt
