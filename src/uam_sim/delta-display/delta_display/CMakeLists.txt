cmake_minimum_required(VERSION 3.0.2)
project(delta_display)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  quadrotor_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
   INCLUDE_DIRS include
)


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/delta_display.cpp
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


add_executable(display_manager src/display_manager.cpp)

target_link_libraries(display_manager
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)



foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
