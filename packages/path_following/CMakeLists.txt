cmake_minimum_required(VERSION 2.8.3)
project(path_following)


find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  sensor_msgs
  rospy
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/path_node.py
  src/planning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
