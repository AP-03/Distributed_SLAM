cmake_minimum_required(VERSION 2.8.3)
project(dataset_ground_truth)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  phasespace_msgs
  rospy
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/phasespace_ground_truth_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
