cmake_minimum_required(VERSION 2.8.3)
project(visual_behaviour)
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_geometry_msgs
  roscpp
  tf2
  tf2_ros
  roslint
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES visual_behaviour
   CATKIN_DEPENDS geometry_msgs tf2_geometry_msgs roscpp tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/visual_behaviour/Movement.cpp
  src/visual_behaviour/PIDController.cpp
  src/visual_behaviour/transforms.cpp
)

set(LINK_LIBS
  visual_behaviour
  ${catkin_LIBRARIES}
)

add_executable(movement_node src/movement_node.cpp)

target_link_libraries(movement_node ${LINK_LIBS})

roslint_cpp(
  src/visual_behaviour/transforms.cpp include/visual_behaviour/transforms.h
  src/movement_node.cpp
)