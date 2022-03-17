cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

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
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_generation
  darknet_ros_msgs
  behaviortree_cpp_v3
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files(
  FILES
  position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS include
  LIBRARIES
  visual_behavior
  geometry_msgs
  tf2_geometry_msgs
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  roslint
  tf
  tf2
  tf2_ros
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_runtime
  behaviortree_cpp_v3
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


add_library(${PROJECT_NAME} 
  src/ball_perception/ColorFilter.cpp
  src/ball_perception/RGBDFilter.cpp
  src/ball_perception/RGBDtf.cpp
  src/human_perception/BBXDetector.cpp
  src/human_perception/BBXToTf.cpp
  src/visual_behavior/Movement.cpp
  src/visual_behavior/PIDController.cpp
  src/visual_behavior/transforms.cpp)

add_library(approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
add_library(detect_object_bt_node SHARED src/behavior_tree_nodes/DetectObject.cpp )
add_library(detect_ball_bt_node SHARED src/behavior_tree_nodes/DetectBall.cpp )
add_library(turn_bt_node SHARED src/behavior_tree_nodes/Turn.cpp)

add_executable(color_filter_node src/color_filter_node.cpp)
target_link_libraries(color_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(rgbd_filter_node src/rgbd_filter_node.cpp)
target_link_libraries(rgbd_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(rgbd_tf_node src/rgbd_tf_node.cpp)
target_link_libraries(rgbd_tf_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bbx_detector_node src/bbx_detector_node.cpp)
target_link_libraries(bbx_detector_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bbx_tf_node src/bbx_tf_node.cpp)
target_link_libraries(bbx_tf_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(movement_node src/movement_node.cpp)
add_executable(follow_1object src/follow_1object.cpp)
add_executable(follow_2objects src/follow_2objects.cpp)
add_executable(follow_1ball src/follow_1ball.cpp)

target_link_libraries(movement_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(follow_1object ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
target_link_libraries(follow_2objects ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
target_link_libraries(follow_1ball ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

list(APPEND plugin_libs
  approach_object_bt_node
  detect_object_bt_node
  detect_ball_bt_node
  turn_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()




roslint_cpp(
  src/color_filter_node.cpp
  src/rgbd_filter_node.cpp
  src/rgbd_tf_node.cpp
  src/bbx_detector_node.cpp
  src/bbx_tf_node.cpp

  src/ball_perception/ColorFilter.cpp
  src/ball_perception/RGBDFilter.cpp
  src/ball_perception/RGBDtf.cpp
  src/human_perception/BBXDetector.cpp
  src/human_perception/BBXToTf.cpp

  include/ball_perception/ColorFilter.h
  include/ball_perception/RGBDFilter.h
  include/ball_perception/RGBDtf.h
  include/human_perception/BBXDetector.h
  include/human_perception/BBXToTf.h
  
  src/visual_behavior/transforms.cpp include/visual_behavior/transforms.h
  src/movement_node.cpp
  src/follow_1object.cpp
  src/follow_1ball.cpp
  src/follow_2objects.cpp
)

set(LINK_LIBS
  visual_behavior
  ${catkin_LIBRARIES}

)