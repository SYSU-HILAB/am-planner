cmake_minimum_required(VERSION 3.0.2)
project(vis_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  decomp_ros_utils
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vis_utils
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs decomp_ros_utils quadrotor_msgs
  DEPENDS
)

include_directories( 
    SYSTEM 
    ${catkin_INCLUDE_DIRS} 
    ${DECOMP_UTIL_INCLUDE_DIRS}
)

add_library(vis_utils
    SHARED
    include/vis_utils/vis_utils.h
)
target_link_libraries(vis_utils
    ${catkin_LIBRARIES}
    ${DECOMP_UTIL_LIBRARIES}
)

set_target_properties(vis_utils PROPERTIES LINKER_LANGUAGE CXX)
