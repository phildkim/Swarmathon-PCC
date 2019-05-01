cmake_minimum_required(VERSION 2.8.3)
project(resource_grid)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcc_srvs
  geometry_msgs
  roscpp
  std_msgs
  std_srvs	
  message_generation
  message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pickup_serv
#  CATKIN_DEPENDS roscpp rospy rostopic std_msgs std_srvs
#  DEPENDS system_lib
   CATKIN_DEPENDS roscpp pcc_srvs std_msgs geometry_msgs
)
add_executable(resources_grid src/grid.cpp)
add_dependencies(resources_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(resources_grid
  ${catkin_LIBRARIES}
)

