cmake_minimum_required(VERSION 3.0.2)
project(limiter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(CATKIN_DEPENDS
  roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gridmap_limiter
  src/gridmap_limiter.cpp
)
add_dependencies(
  gridmap_limiter
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gridmap_limiter ${catkin_LIBRARIES})

add_executable(pointcloud_limiter
  src/pointcloud_limiter.cpp
)
add_dependencies(
  pointcloud_limiter
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pointcloud_limiter ${catkin_LIBRARIES})

install(
  TARGETS
  gridmap_limiter
  pointcloud_limiter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
