Robot Localization Project
This project provides robot localization functionality using ROS. It is compatible with both ROS Melodic (Ubuntu 18.04) and ROS Noetic (Ubuntu 20.04).

Supported ROS versions: Melodic (Ubuntu 18.04) and Noetic (Ubuntu 20.04).

Install the required dependencies:

bash
Kopieren
sudo apt-get install libgeographic-dev
sudo apt-get install ros-$(rosversion -d)-geographic-msgs
Use the standard ROS catkin build process. First, navigate to your catkin workspace:

bash
Kopieren
cd ~/catkin_ws
Then, clone or place this repository in the src folder:

bash
Kopieren
cd ~/catkin_ws/src
git clone <this-repository-url>
Finally, build the workspace:

bash
Kopieren
cd ~/catkin_ws
catkin build
Within the data_publish package, there is a data folder. You need to copy and paste the dataset folders/files from data_publish/data into the same data folder to ensure they are in the correct location.

For example, your folder structure should look like this:

kotlin
Kopieren
data_publish/
 ┣━ data/
    ┣━ folder1/
    ┣━ folder2/
    ┗━ ...
Make sure all required data files/folders are placed inside the data directory.

To run the robot localization node, use the following command:

bash
Kopieren
roslaunch robot_localization ekf.launch
This command starts the Extended Kalman Filter (EKF) for localization.

Feel free to open issues or submit pull requests to help improve this project.

This project is distributed under the MIT License.

