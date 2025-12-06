cmake_minimum_required(VERSION 3.0.2)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(pybind11 REQUIRED)
find_package(autodiff REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_server
  jps3d
  decomp_ros_utils
  quadrotor_msgs
  visualization_msgs
  vis_utils
  message_generation
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${pybind11_INCLUDE_DIRS}
)

add_service_files(FILES forward_srv.srv)
add_service_files(FILES workspace_probability_srv.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES se3_planner
  CATKIN_DEPENDS roscpp rospy octomap_server decomp_ros_utils quadrotor_msgs vis_utils jps3d message_runtime
  DEPENDS EIGEN3
)

add_library(se3_planner
    src/se3_planner.cc
)

target_link_libraries(se3_planner
    ${catkin_LIBRARIES}
    autodiff::autodiff
)
