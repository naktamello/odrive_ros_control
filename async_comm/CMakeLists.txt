cmake_minimum_required(VERSION 2.8.3)
project(async_comm)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package(
INCLUDE_DIRS
  include
DEPENDS
  Boost
LIBRARIES
  can_device
  )

add_definitions(-DLINUXSOCKETS=1)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(can_device src/async_can.cpp)
target_link_libraries(can_device ${Boost_LIBRARIES})

install(
  TARGETS can_device
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)