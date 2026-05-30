cmake_minimum_required(VERSION 2.8.3)
project(dataset_validation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/imu_marker_visualizer.py
  scripts/odom_to_path.py
  scripts/validate_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
