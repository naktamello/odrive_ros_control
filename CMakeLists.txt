cmake_minimum_required(VERSION 2.8.3)
project(odrive_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  control_toolbox
  transmission_interface
  pluginlib
  urdf
  angles
  gazebo_dev
  simple_message
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros_control
    controller_manager
    hardware_interface
    joint_limits_interface
    realtime_tools
    roscpp
    std_msgs
    control_toolbox
    transmission_interface
    pluginlib
    urdf
    angles
    industrial_robot_client
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    odrive_hardware_interface
    odrive_hardware_sim
)

add_definitions(-DLINUXSOCKETS=1)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(odrive_hardware_interface src/odrive_hardware_interface.cpp)
target_link_libraries(odrive_hardware_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(odrive_hardware_sim src/odrive_hardware_sim.cpp)
target_link_libraries(odrive_hardware_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(odrive_hardware_interface_node src/odrive_hardware_interface_node.cpp)
target_link_libraries(odrive_hardware_interface_node odrive_hardware_interface)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_motion_streamer_interface src/odrive_joint_streamer.cpp)
target_link_libraries(${PROJECT_NAME}_motion_streamer_interface
  industrial_robot_client industrial_utils simple_message ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_streamer_interface
  PROPERTIES OUTPUT_NAME motion_streamer_interface PREFIX "")

install(
  TARGETS odrive_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  TARGETS odrive_hardware_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(
  # TARGETS odrive_hardware_sim
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES odrive_hardware_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

