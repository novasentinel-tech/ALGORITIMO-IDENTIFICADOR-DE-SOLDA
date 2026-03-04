cmake_minimum_required(VERSION 3.0.2)
project(welding_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  moveit_core
  moveit_ros_planning_interface
  trajectory_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    cv_bridge 
    moveit_core 
    moveit_ros_planning_interface
    trajectory_msgs
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Install python scripts
catkin_install_python(
  PROGRAMS
    scripts/image_publisher_node.py
    scripts/welding_planner_node.py
    scripts/movement_executor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Enable testing
enable_testing()

# Add test configuration
if(CATKIN_ENABLE_TESTING)
  install(
    DIRECTORY test/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  )
endif()
