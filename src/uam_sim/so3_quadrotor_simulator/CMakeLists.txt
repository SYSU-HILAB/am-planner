cmake_minimum_required(VERSION 2.8.3)
project(so3_quadrotor_simulator)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  geometry_msgs
  rosbag
  # uav_utils
)


find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ode)
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp quadrotor_msgs geometry_msgs
  DEPENDS Eigen3 system_lib
)




add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp)

target_link_libraries(quadrotor_simulator_so3 
   ${catkin_LIBRARIES}
   quadrotor_dynamics
)


add_executable(test_quadrotor src/test_quadrotor.cpp)
target_link_libraries(test_quadrotor
   ${catkin_LIBRARIES}
)