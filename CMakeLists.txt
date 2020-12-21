cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hunter_msgs
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  tf2
  tf
  tf2_ros
  rospy
  dynamic_reconfigure
)

find_package(orocos_kdl REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  ackermann_msgs 
  tf2 
  tf
  tf2_ros 
  rospy 
  dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES hunter_messenger
)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pure_pursuit src/pure_pursuit.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
