cmake_minimum_required(VERSION 3.0.2)
project(week2_pkg)

## 캐킨 빌드를 할 때 요구되는 구성요소 패키지이다.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  turtlesim
  visualization_msgs
)

## 캐킨 패키지 옵션으로 라이브러리, 캐킨 빌드 의존성, 시스템 의존 패키지를 기술한다.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES week2_pkg
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim
#  DEPENDS system_lib
)

## include 디렉터리를 설정한다.
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## week2_node 노드에 대한 빌드 옵션이다.
## 실행 파일, 타킷 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(week2_node src/week2_node.cpp)
add_dependencies(week2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week2_node
  ${catkin_LIBRARIES}
)

add_executable(week2_node_practice src/week2_node_practice.cpp)
add_dependencies(week2_node_practice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week2_node_practice
  ${catkin_LIBRARIES}
)