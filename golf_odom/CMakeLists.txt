cmake_minimum_required(VERSION 2.8.3)
project(golf_odom)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
  tf
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES golf_odom
#  CATKIN_DEPENDS geometry_msgs roscpp rospy serial std_msgs tf nav_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(golf_odom src/golf_odom.cpp)
target_link_libraries(golf_odom ${catkin_LIBRARIES})
