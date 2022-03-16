cmake_minimum_required(VERSION 2.8.3)
project(visual_behaviour)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")
set(CMAKE_CXX_FLAGS "-std=c++17")


find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_geometry_msgs
  roscpp
  tf2
  tf2_ros
  roslint
  std_msgs
  kobuki_msgs
  sensor_msgs
  behaviortree_cpp_v3
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
    visual_behaviour
   CATKIN_DEPENDS 
    geometry_msgs 
    tf2_geometry_msgs 
    roscpp 
    tf2 
    tf2_ros
    roslint
    std_msgs
    kobuki_msgs
    sensor_msgs
    behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
add_library(calculate_distance_bt_node SHARED src/behavior_tree_nodes/CalculateDistance.cpp)
add_library(detect_object_bt_node SHARED src/behavior_tree_nodes/DetectObject.cpp )
add_library(turn_bt_node SHARED src/behavior_tree_nodes/Turn.cpp)

list(APPEND plugin_libs
  approach_object_bt_node
  calculate_distance_bt_node
  detect_object_bt_node
  turn_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

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
add_executable(follow_1object src/follow_1object.cpp)
add_executable(follow_2objects src/follow_2objects.cpp)

target_link_libraries(movement_node ${LINK_LIBS})
target_link_libraries(follow_1object ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
target_link_libraries(follow_2objects ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/visual_behaviour/transforms.cpp include/visual_behaviour/transforms.h
  src/movement_node.cpp
  src/follow_1object.cpp
  src/follow_2objects.cpp
)