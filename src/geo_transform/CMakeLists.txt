cmake_minimum_required(VERSION 3.0.2)
project(geo_transform)

## 使用C++11标准(如有需要)
add_compile_options(-std=c++11)

## 寻找catkin和依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # 如果需要tf或tf2则在这里加入
  # tf
  # tf2
)

## 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geo_transform
  CATKIN_DEPENDS roscpp
  # 如果用了tf库就加入tf或tf2
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 编译库文件
add_library(geo_transform
  src/geo_transform.cpp
)

target_link_libraries(geo_transform
  ${catkin_LIBRARIES}
)

## 如果你有测试或示例节点(可选)
# add_executable(test_node src/test_node.cpp)
# target_link_libraries(test_node ${catkin_LIBRARIES} geo_transform)

## 安装目标(可选)
install(TARGETS geo_transform
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
