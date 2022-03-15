cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_generation
  darknet_ros_msgs
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
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  sensor_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
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
)

add_executable(color_filter_node src/color_filter_node.cpp)
target_link_libraries(color_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(rgbd_filter_node src/rgbd_filter_node.cpp)
target_link_libraries(rgbd_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(rgbd_tf_node src/rgbd_tf_node.cpp)
target_link_libraries(rgbd_tf_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bbx_detector_node src/bbx_detector_node.cpp)
target_link_libraries(bbx_detector_node ${PROJECT_NAME} ${catkin_LIBRARIES})


roslint_cpp(
  src/color_filter_node.cpp
  src/rgbd_filter_node.cpp
  src/rgbd_tf_node.cpp
  src/bbx_detector_node.cpp

  src/ball_perception/ColorFilter.cpp
  src/ball_perception/RGBDFilter.cpp
  src/ball_perception/RGBDtf.cpp

  include/ball_perception/ColorFilter.h
  include/ball_perception/RGBDFilter.h
  include/ball_perception/RGBDtf.h
)


# catkin_add_gtest(Nombretest tests/Nombretest.cpp)
# add_dependencies(Nombretest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# if(TARGET Nombretest)
#    target_link_libraries(Nombretest ${catkin_LIBRARIES} ${PROJECT_NAME})
#  endif()

