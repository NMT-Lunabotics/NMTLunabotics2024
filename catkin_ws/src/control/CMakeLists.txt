cmake_minimum_required(VERSION 3.0.2)
project(control)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  actionlib
  move_base_msgs
  tf
  geometry_msgs
  actions
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs actionlib move_base_msgs tf geometry_msgs actions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dump_node src/dump_node.cpp)

target_link_libraries(dump_node
  ${catkin_LIBRARIES}
)

add_dependencies(dump_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
