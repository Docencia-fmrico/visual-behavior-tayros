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
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)


add_executable(color_filter_node src/color_filter_node.cpp)
target_link_libraries(color_filter_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable()


# target_link_libraries(node_name ${catkin_LIBRARIES} ${PROJECT_NAME})


roslint_cpp(
  src/color_filter_node.cpp
)


# catkin_add_gtest(Nombretest tests/Nombretest.cpp)
# add_dependencies(Nombretest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# if(TARGET Nombretest)
#    target_link_libraries(Nombretest ${catkin_LIBRARIES} ${PROJECT_NAME})
#  endif()

