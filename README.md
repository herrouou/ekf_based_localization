# ROS Robot Localization

This project is based on ROS and supports the following versions:
- **Melodic** (Ubuntu 18.04)
- **Noetic** (Ubuntu 20.04)

## Environment Setup

Ensure that you have the corresponding ROS version (Melodic or Noetic) installed along with [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/).

install the following dependencies:
   ```bash
   sudo apt-get install libgeographic-dev
   ```bash
   sudo apt-get install ros-$(rosversion -d)-geographic-msgs

## Build Instructions

1. Clone this repository into your catkin workspace (e.g., `~/catkin_ws/src`).
2. Navigate to the workspace root directory:
   ```bash
   cd ~/catkin_ws
