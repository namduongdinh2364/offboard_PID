cmake_minimum_required(VERSION 3.0.2)
project(offboard_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/pid_controller.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(offboard_pid
  src/pid_controller_base.cpp
)

file(GLOB SOURCES
  src/controller_uav.cpp
  src/controller_uav_node.cpp
  src/pid_controller_base.cpp
)

## Add cmake target dependencies of the executable
add_executable(controller_uav_node ${SOURCES})
target_link_libraries(controller_uav_node ${catkin_LIBRARIES} offboard_pid)

add_executable(setmode_offb_node src/setmode_offb.cpp)
target_link_libraries(setmode_offb_node ${catkin_LIBRARIES})


add_executable(start_landing_service
  src/service_land.cpp
)
target_link_libraries(start_landing_service ${catkin_LIBRARIES})

# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
