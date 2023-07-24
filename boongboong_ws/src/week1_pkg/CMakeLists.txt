cmake_minimum_required(VERSION 3.0.2)
project(week1_pkg)

## 캐킨 빌드를 할 때 요구되는 구성요소 패키지이다.
## 의존성 패키지로 message_generation, std_msgs, roscpp이며 이 패키지들이 존재하지 않으면 빌드도중에 에러가 난다.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## 캐킨 패키지 옵션으로 라이브러리, 캐킨 빌드 의존성, 시스템 의존 패키지를 기술한다.
catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES ros_tutorials_topic
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## include 디렉터리를 설정한다.
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

######################################################################################################
## week1_node2 노드에 대한 빌드 옵션이다.
## 실행 파일, 타킷 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(week1_node2 src/week1_node2.cpp)
add_dependencies(week1_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week1_node2
  ${catkin_LIBRARIES}
)

## week1_node1 노드에 대한 빌드 옵션이다.
add_executable(week1_node1 src/week1_node1.cpp)
add_dependencies(week1_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week1_node1
  ${catkin_LIBRARIES}
)

######################################################################################################
## week1_node2 노드에 대한 빌드 옵션이다.
## 실행 파일, 타킷 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(week0_node2 src/week0_node2.cpp)
add_dependencies(week0_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week0_node2
  ${catkin_LIBRARIES}
)

## week1_node2 노드에 대한 빌드 옵션이다.
## 실행 파일, 타킷 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(week0_node1 src/week0_node1.cpp)
add_dependencies(week0_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(week0_node1
  ${catkin_LIBRARIES}
)