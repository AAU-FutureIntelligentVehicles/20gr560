cmake_minimum_required(VERSION 3.0.2)
project(realsense)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy

)

find_package(realsense2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)


add_executable(pose_tracker src/pose_tracker.cpp)
target_link_libraries(pose_tracker ${catkin_LIBRARIES} ${realsense2_LIBRARY})
