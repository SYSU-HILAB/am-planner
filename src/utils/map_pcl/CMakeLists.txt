cmake_minimum_required(VERSION 3.0.2)
project(map_pcl)

add_compile_options(-std=c++11)
add_compile_options(-std=c++14)


find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(visual_pcl
  src/visual_pcl.cpp
)

target_link_libraries(visual_pcl
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

