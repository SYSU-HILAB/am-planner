cmake_minimum_required(VERSION 3.10.0)
project(plan_manage)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 使用现代CMake的方式自动查找Python
# 这会利用`conda activate`设置的环境变量，自动找到正确的Python
find_package(Python 3 REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Found Python executable: ${Python_EXECUTABLE}")
message(STATUS "Found Python include dirs: ${Python_INCLUDE_DIRS}")
message(STATUS "Found Python libraries: ${Python_LIBRARIES}")

# 查找pybind11。它会自动使用上面找到的Python版本
find_package(pybind11 REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# 设置pybind11查找参数
set(PYBIND11_PYTHON_VERSION "3" CACHE STRING "")
set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})

# 添加pybind11查找
find_package(pybind11 REQUIRED)
find_package(autodiff REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  traj_opt
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
)

catkin_package()

add_executable(se3_node
    src/plan_manage.cpp
)

target_link_libraries(se3_node
  ${catkin_LIBRARIES}
  pybind11::embed
  autodiff::autodiff
)

# 确保编译时能找到Python.h
target_include_directories(se3_node PRIVATE ${PYTHON_INCLUDE_DIRS})