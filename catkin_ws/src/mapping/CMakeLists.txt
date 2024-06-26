cmake_minimum_required(VERSION 2.8.3)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    roslaunch
    grid_map_ros
    grid_map_msgs
    tf
    apriltag_ros
    visualization_msgs
)
roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS
    std_msgs
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    grid_map_ros
    grid_map_msgs
    tf
    apriltag_ros
    visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(pose_pub src/pose_pub.cpp)
add_dependencies(pose_pub
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_pub ${catkin_LIBRARIES})

add_executable(pc_pub src/pc_pub.cpp)
add_dependencies(pc_pub
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pc_pub ${catkin_LIBRARIES})

add_executable(cvt_occupancy src/cvt_occupancy.cpp)
add_dependencies(cvt_occupancy
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cvt_occupancy ${catkin_LIBRARIES})

add_executable(april_tag_loc src/april_tag_loc.cpp)
add_dependencies(april_tag_loc
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(april_tag_loc ${catkin_LIBRARIES})

add_executable(create_occupancy src/create_occupancy.cpp)
add_dependencies(create_occupancy
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_occupancy ${catkin_LIBRARIES})

add_executable(marker_node src/marker_node.cpp)
add_dependencies(marker_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_node ${catkin_LIBRARIES})

install(TARGETS
    pose_pub
    pc_pub
    cvt_occupancy
    april_tag_loc
    create_occupancy
    marker_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
