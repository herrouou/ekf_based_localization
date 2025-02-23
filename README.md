# ROS Robot Localization Package

This repository contains a ROS-based robot localization package of the sensor fusion for localization using iPhone. 

## ROS Version
- **ROS Melodic** (Ubuntu 18.04)
- **ROS Noetic** (Ubuntu 20.04)

## Dependencies Installation

Before building the package, you need to install the required dependencies. Run the following commands in your terminal:

```bash
sudo apt-get install libgeographic-dev
sudo apt-get install ros-$(rosversion -d)-geographic-msgs
```
## Clone and Building the Package

Firstly clone the following code:
```bash
git clone https://github.com/herrouou/ekf_based_localization.git
```
Then navigate to your the workspace:
```bash
cd ekf_based_localization
```
Andn build the package using catkin_make or catkin build. Run:

```bash
catkin_make
```
or
```bash
catkin build
```
## Data folder changing

Within the data_publish package, there is a data folder.

```bash
data_publish/
 ┣━ data/
    ┣━ folder1/
    ┣━ folder2/
    ┗━ ...
```
Copy files in one of the folders and paste them into the data folder.

## Configuration
The configuration file is in ``` robot_lacalization/config ```, ``` ekf_global.yaml``` is only for global fusion, ```ekf_local``` is only for local fusion. ```ekf_local_global_common.yaml``` is for common settings.

## Run the EKF Fusion

```bash
 roslaunch robot_localization ekf.launch
```
