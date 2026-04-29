cmake_minimum_required(VERSION 2.8.3)
project(dataset_ground_truth)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  phasespace_msgs
  rosbag
  rospy
  std_msgs
  tf
  tf2_ros
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mocap_debug_summary.py
  scripts/mocap_debug_visualizer.py
  scripts/phasespace_ground_truth_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
