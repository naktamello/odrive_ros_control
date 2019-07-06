cmake_minimum_required(VERSION 2.8.3)
project(async_comm)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS
system
thread
)

catkin_package(
INCLUDE_DIRS
  include
DEPENDS
  Boost
LIBRARIES
  ${PROJECT_NAME}_can
  ${PROJECT_NAME}_serial
  )

add_definitions(-DLINUXSOCKETS=1)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_can src/async_can.cpp)
target_link_libraries(${PROJECT_NAME}_can ${Boost_LIBRARIES})
add_library(${PROJECT_NAME}_serial src/async_serial.cpp)
target_link_libraries(${PROJECT_NAME}_serial ${Boost_LIBRARIES})


install(
  TARGETS ${PROJECT_NAME}_can ${PROJECT_NAME}_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)