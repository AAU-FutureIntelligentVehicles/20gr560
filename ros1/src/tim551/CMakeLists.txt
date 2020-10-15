cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(tim551)

find_package(catkin REQUIRED
    COMPONENTS
      diagnostic_updater
      dynamic_reconfigure
      roscpp
      rospy
      roslaunch
      sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    DEPENDS Boost libusb
    CATKIN_DEPENDS roscpp rospy sensor_msgs diagnostic_updater dynamic_reconfigure
    LIBRARIES
    INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${libusb_INCLUDE_DIRS})

add_executable(scanner1 src/scanner1.cpp)
target_link_libraries(scanner1 ${catkin_LIBRARIES})


