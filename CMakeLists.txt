cmake_minimum_required(VERSION 3.0.2)
project(server_clean_robot_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  actionlib
  move_base_msgs
  gazebo_ros
  robot_state_publisher
  joint_state_publisher
  xacro
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  actionlib
  move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_demo src/navigation_demo.cpp)

add_dependencies(navigation_demo ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigation_demo
  ${catkin_LIBRARIES}
)
