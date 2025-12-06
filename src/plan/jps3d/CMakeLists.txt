cmake_minimum_required(VERSION 2.8.3)
project(jps3d)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  octomap_ros
  octomap_msgs
  octomap_server
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES jps3d
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)


add_library( jps3d 
    src/graph_search.cpp
    src/jps_planner.cpp
    )
target_link_libraries( jps3d
    ${catkin_LIBRARIES} 
    )  