cmake_minimum_required(VERSION 2.8.3)
project(slam_comparison)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gt_comparison_node.py
  scripts/preprocess_bags.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
