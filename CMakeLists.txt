cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    roslint
    control_toolbox
    dynamic_reconfigure
    nav_msgs
    tf
    )


generate_dynamic_reconfigure_options(
  cfg/PidConfig.cfg
)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    pluginlib
    roscpp
    roslint
    control_toolbox
    dynamic_reconfigure
    nav_msgs
    tf
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME} SHARED
    src/HeroChassisController.cpp
    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    )

install(
    FILES hero_chassis_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

roslint_cpp()
