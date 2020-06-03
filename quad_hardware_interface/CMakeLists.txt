cmake_minimum_required(VERSION 2.8.3)
project(quad_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
	
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
		
  LIBRARIES 
    quad_hardware_interface
		
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(quad_hardware_interface src/quad_hardware_interface.cpp)
target_link_libraries(quad_hardware_interface
	${catkin_LIBRARIES})

add_executable(quad_hardware_interface_node src/quad_hardware_interface_node.cpp)
add_dependencies(quad_hardware_interface_node quad_hardware_interface)

target_link_libraries(quad_hardware_interface_node
	quad_hardware_interface
	${catkin_LIBRARIES})
