cmake_minimum_required(VERSION 2.8.3)
project(actions)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    can_raw
    can_convert
)

add_service_files(
    FILES
    execute_dumping_autonomy.srv
    execute_digging_autonomy.srv
    execute_script.srv
)
generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS
    std_msgs
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(dumping_autonomy src/dumping_autonomy_node.cpp)
add_dependencies(dumping_autonomy
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dumping_autonomy ${catkin_LIBRARIES})

add_executable(digging_autonomy src/digging_autonomy_node.cpp)
add_dependencies(digging_autonomy
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(digging_autonomy ${catkin_LIBRARIES})

add_executable(funkenstein src/funkenstein.cpp)
add_dependencies(funkenstein
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(funkenstein ${catkin_LIBRARIES})

install(TARGETS
    dumping_autonomy
    digging_autonomy
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
