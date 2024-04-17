cmake_minimum_required(VERSION 3.0.2)
project(camera_stream)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  theora_image_transport
)

catkin_package(CATKIN_DEPENDS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  theora_image_transport
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(camera_stream_node
  src/camera_stream_node.cpp
)
add_dependencies(
  camera_stream_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(camera_stream_node ${catkin_LIBRARIES})

install(
  TARGETS camera_stream_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
