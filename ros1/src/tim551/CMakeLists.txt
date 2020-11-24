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
      message_generation
      visualization_msgs
      tf
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
    FILES
 #   floater.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
)


catkin_package(
    DEPENDS Boost libusb
    CATKIN_DEPENDS roscpp rospy sensor_msgs diagnostic_updater dynamic_reconfigure message_runtime visualization_msgs tf
    LIBRARIES
    INCLUDE_DIRS)
include_directories(include ${catkin_INCLUDE_DIRS} ${libusb_INCLUDE_DIRS})


add_executable(scanner1 src/scanner1.cpp)
target_link_libraries(scanner1 ${catkin_LIBRARIES})
add_dependencies(scanner1 tim551_generate_messages_cpp)
add_executable(avgsub src/avgsub.cpp)
target_link_libraries(avgsub ${catkin_LIBRARIES})
add_dependencies(avgsub tim551_generate_messages_cpp)
add_executable(tfe src/tfe.cpp)
target_link_libraries(tfe ${catkin_LIBRARIES})
add_dependencies(tfe tim551_generate_messages_cpp)
add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES})
add_dependencies(test1 tim551_generate_messages_cpp)


