################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(fortyfive_robot_example)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  interactive_markers
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  actionlib_msgs
  fortyfive_robot_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_action_files(
  FILES
  Fortyfive_robot.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    interactive_markers
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    actionlib_msgs
    fortyfive_robot_msgs
    message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  nodes/fortyfive_robot_obstacle
  nodes/fortyfive_robot_client
  nodes/fortyfive_robot_server
  nodes/fortyfive_robot_pointop_key
  nodes/fortyfive_robot_bumper 
  nodes/fortyfive_robot_cliff
  nodes/fortyfive_robot_sonar
  nodes/fortyfive_robot_illumination
  nodes/fortyfive_robot_marker_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
