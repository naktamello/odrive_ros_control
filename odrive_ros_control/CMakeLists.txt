cmake_minimum_required(VERSION 2.8.3)
project(odrive_ros_control)

add_compile_options(-std=c++14)

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
  message_generation
  async_comm
)

find_package(Boost REQUIRED COMPONENTS
system
thread
)

add_service_files(
   FILES
   ODriveRawCAN.srv
   SetReadMode.srv
   SetWriteMode.srv
   SetRequestedState.srv
   GetAxisError.srv
   GetCurrentState.srv
   GetVbusVoltage.srv
   SetPosSetpoint.srv
   SetVelSetpoint.srv
   SetCurrentSetpoint.srv
   MoveToPos.srv
)


generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
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
    async_comm
  INCLUDE_DIRS
    include
  DEPENDS
    Boost
  LIBRARIES
    odrive_hardware_interface
    odrive_hardware_sim
)




add_definitions(-DLINUXSOCKETS=1)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(odrive_hardware_interface src/odrive_hardware_interface.cpp)
add_dependencies(odrive_hardware_interface odrive_ros_control_generate_messages_cpp)
target_link_libraries(odrive_hardware_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(odrive_hardware_sim src/odrive_hardware_sim.cpp)
target_link_libraries(odrive_hardware_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(odrive_transport_uart src/odrive_transport_uart.cpp)
add_dependencies(odrive_transport_uart odrive_ros_control_generate_messages_cpp)
target_link_libraries(odrive_transport_uart ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(odrive_transport_can src/odrive_transport_can.cpp)
add_dependencies(odrive_transport_can odrive_ros_control_generate_messages_cpp)
target_link_libraries(odrive_transport_can ${catkin_LIBRARIES} ${Boost_LIBRARIES})

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
  TARGETS odrive_hardware_interface odrive_hardware_sim odrive_transport_uart odrive_transport_can
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES odrive_hardware_sim_plugin.xml odrive_transport_plugin.xml odrive_hardware_interface_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
