cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

find_package(catkin REQUIRED COMPONENTS
            robot_state_publisher
            urdf
            rviz
            rospy
            roscpp
            sensor_msgs
            tf
            geometry_msgs
)

catkin_package()

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf    
)

