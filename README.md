# ROS Robot Localization Package

This repository contains a ROS-based robot localization package that supports both ROS Melodic (Ubuntu 18.04) and ROS Noetic (Ubuntu 20.04). The package utilizes an Extended Kalman Filter (EKF) for sensor fusion and localization.

## ROS Version
- **ROS Melodic** (Ubuntu 18.04)
- **ROS Noetic** (Ubuntu 20.04)

## Dependencies Installation

Before building the package, you need to install the required dependencies. Run the following commands in your terminal:

```bash
sudo apt-get install libgeographic-dev
sudo apt-get install ros-$(rosversion -d)-geographic-msgs
```
## Building the Package

After installing the dependencies, you can build the package using catkin_make or catkin build. Navigate to your catkin workspace and run:

```bash
catkin_make
```
or

```bash
catkin build
```
