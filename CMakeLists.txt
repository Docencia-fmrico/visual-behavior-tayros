cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(perception_node src/perception_node.cpp)


# add_executable()


# target_link_libraries(node_name ${catkin_LIBRARIES} ${PROJECT_NAME})


roslint_cpp(
  src/perception_node.cpp
)


# catkin_add_gtest(Nombretest tests/Nombretest.cpp)
# add_dependencies(Nombretest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# if(TARGET Nombretest)
#    target_link_libraries(Nombretest ${catkin_LIBRARIES} ${PROJECT_NAME})
#  endif()

