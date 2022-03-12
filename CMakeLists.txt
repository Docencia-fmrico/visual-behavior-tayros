cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
  behaviortree_cpp_v3
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs sensor_msgs behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
add_library(calculate_distance_bt_node SHARED src/behavior_tree_nodes/CalculateDistance.cpp)
add_library(detect_object_bt_node SHARED src/behavior_tree_nodes/DetectObject.cpp )
add_library(is_something_near_bt_node SHARED src/behavior_tree_nodes/IsSomethingNear.cpp)
add_library(turn_bt_node SHARED src/behavior_tree_nodes/Turn.cpp)
add_library(dodge_obstacle_node SHARED src/behavior_tree_nodes/DodgeObstacle.cpp)

list(APPEND plugin_libs
  approach_object_bt_node
  calculate_distance_bt_node
  detect_object_bt_node
  is_something_near_bt_node
  turn_bt_node
  dodge_obstacle_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(perception_node src/perception_node.cpp)
add_executable(follow_human src/follow_human.cpp)

target_link_libraries(follow_human ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})


# add_executable()


# target_link_libraries(node_name ${catkin_LIBRARIES} ${PROJECT_NAME})


roslint_cpp(
  src/perception_node.cpp
  src/follow_human.cpp
)


# catkin_add_gtest(Nombretest tests/Nombretest.cpp)
# add_dependencies(Nombretest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# if(TARGET Nombretest)
#    target_link_libraries(Nombretest ${catkin_LIBRARIES} ${PROJECT_NAME})
#  endif()

