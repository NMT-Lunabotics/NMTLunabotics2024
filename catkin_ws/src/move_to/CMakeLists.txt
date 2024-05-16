cmake_minimum_required(VERSION 3.0.2)
project(move_to)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_service_files(
  FILES
  move_to.srv
)
generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS
  roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_to_node
  src/move_to_node.cpp
)
add_dependencies(
  move_to_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(move_to_node ${catkin_LIBRARIES})

install(
  TARGETS move_to_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
