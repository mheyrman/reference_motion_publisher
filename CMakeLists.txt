cmake_minimum_required(VERSION 3.0.2)
project(reference_motion_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
)

find_package(nlohmann_json REQUIRED)
# Eigen
find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reference_motion_publisher
  CATKIN_DEPENDS roscpp std_msgs
)

catkin_install_python(PROGRAMS
  src/reference_motion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(reference_motion_publisher src/reference_motion_pub.cpp)
target_link_libraries(reference_motion_publisher ${catkin_LIBRARIES} nlohmann_json::nlohmann_json)
target_include_directories(reference_motion_publisher PRIVATE ${catkin_INCLUDE_DIRS})

