cmake_minimum_required(VERSION 2.8.3)
project(tube_navigation)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  ropod_ros_msgs
  geometry_msgs
  actionlib_msgs
  dynamic_reconfigure
  tf
)


generate_dynamic_reconfigure_options(
    config/TubeNavigation.cfg
)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(tube_navigation
  src/tube_navigation_ros.cpp
)


target_link_libraries(tube_navigation
  ${catkin_LIBRARIES}
)