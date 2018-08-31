cmake_minimum_required(VERSION 2.8.3)
project(maneuver_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  tf
  costmap_2d
  maneuver_planner
  pluginlib
  nav_core
  ed
  maneuver_navigation
)

add_message_files(
   DIRECTORY msg
   FILES
   goal.msg
   configuration.msg
   feedback.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES maneuver_navigation
  CATKIN_DEPENDS  message_runtime std_msgs geometry_msgs nav_msgs roscpp  tf costmap_2d maneuver_planner
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(maneuver_navigation src/maneuver_navigation_rosnode.cpp  src/maneuver_navigation.cpp)
target_link_libraries(maneuver_navigation ${catkin_LIBRARIES})
add_dependencies(maneuver_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(maneuver_navigationED src/maneuver_navigation_rosnode.cpp  src/maneuver_navigation.cpp src/maneuver_navigation_ed.cpp)
# target_link_libraries(maneuver_navigation ${catkin_LIBRARIES})
# add_dependencies(maneuver_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
