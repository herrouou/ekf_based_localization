# ekf_based_localization

ubuntu 18 + ROS medodic, and it uses the source code of robot_localization  
catkin build  
sorce devel/setup.bash  
roslaunch robot_localization ekf.launch  
**data_publish is a package which publsihes GPS, imu and vio raw data**
**path_publish can generate the paths of ground truth, raw gps and fused data**
