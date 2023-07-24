cmake_minimum_required(VERSION 3.0.2)
project(week5_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES week5_pkg
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include                 // hpp 파일을 따로 사용하기 위해 주석을 풀어준다. hpp, cpp파일 분리
  ${catkin_INCLUDE_DIRS}
)

## week2_node 노드에 대한 빌드 옵션이다.
## 실행 파일, 타킷 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(week5_node src/week5_node.cpp)
add_dependencies(week5_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week5_node
  ${catkin_LIBRARIES}
)
