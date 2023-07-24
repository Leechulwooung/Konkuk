cmake_minimum_required(VERSION 3.0.2)
project(week4_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES week4_pkg
#  CATKIN_DEPENDS roscpp rospy tf turtlesim
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})