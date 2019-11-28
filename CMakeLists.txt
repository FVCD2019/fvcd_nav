cmake_minimum_required(VERSION 2.8.3)
project(fvcd_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  move_base
  tf
  geometry_msgs
  nav_msgs
  rospy
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fvcd_nav
#  CATKIN_DEPENDS move_base tf2
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/odom_b.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

