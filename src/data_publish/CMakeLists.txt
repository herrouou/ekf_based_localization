cmake_minimum_required(VERSION 3.0.2)
project(data_publish)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag
  cmake_modules
  nav_msgs  
  tf2_ros
  geo_transform
)
find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})





catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rosbag nav_msgs tf2_ros
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_executable(sensor_data_publish src/sensor_data_publish.cpp)
add_executable(kml_visual src/kml_visual.cpp)
add_executable(vio_visual src/vio_visual.cpp)
add_executable(gps_enu_node src/gps_enu_node.cpp)

target_link_libraries(sensor_data_publish
  ${catkin_LIBRARIES}
  
)


target_link_libraries(kml_visual
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

target_link_libraries(vio_visual
  ${catkin_LIBRARIES}
  
)

target_link_libraries(gps_enu_node
  ${catkin_LIBRARIES}
)



