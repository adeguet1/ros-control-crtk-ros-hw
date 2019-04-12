cmake_minimum_required (VERSION 2.8.3)

project (ros_control_crtk_ros_hw)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  crtk_msgs
)

catkin_package (
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    actionlib
    control_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    trajectory_msgs
    crtk_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories (include ${catkin_INCLUDE_DIRS})

# library for hardware interface
add_library (ros_control_crtk_ros_hw src/crtk_ros_hardware_interface.cpp)
target_link_libraries (ros_control_crtk_ros_hw
                       ${catkin_LIBRARIES})

# main executable, i.e. ROS node
add_executable (crtk_ros_hw_controller src/crtk_ros_hw_controller.cpp)
target_link_libraries (crtk_ros_hw_controller ros_control_crtk_ros_hw ${catkin_LIBRARIES})

# install
install (DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install (DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# install executables and/or libraries
install (TARGETS ros_control_crtk_ros_hw crtk_ros_hw_controller
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install (DIRECTORY include/${PROJECT_NAME}/
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
         FILES_MATCHING PATTERN "*.h")
