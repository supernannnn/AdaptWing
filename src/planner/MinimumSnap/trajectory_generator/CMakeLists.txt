cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
)


find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
  src/trajectory_generator_waypoint.cpp
)


target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
)