cmake_minimum_required(VERSION 3.0.2)
project(console)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  traj_utils
  visual_utils
  tf
  cv_bridge
  image_transport
  message_generation
  circle_detection
)

add_message_files(
  FILES
  ConsoleState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS  visual_utils tf traj_utils message_runtime circle_detection
)


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(console_node 
	src/console_node.cpp
  src/minimumsnap.cpp
  src/console_FSM.cpp
)


target_link_libraries(console_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)