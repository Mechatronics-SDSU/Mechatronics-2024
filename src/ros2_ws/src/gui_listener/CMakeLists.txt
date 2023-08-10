cmake_minimum_required(VERSION 3.5)

project(gui_listener_node VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(STYLESHEET_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/styles.qss")
add_definitions(-DSTYLE_SHEET_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/styles.qss")

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

add_subdirectory(${ROOT_DIR}/classes/src/dependency_library ${CMAKE_BINARY_DIR}/classes-build)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(gui_listener_exec 
    src/main.cpp
    src/gui_listener_node.cpp
)

target_include_directories(gui_listener_exec PUBLIC include)

ament_target_dependencies(gui_listener_exec PUBLIC rclcpp scion_types std_msgs)

target_link_libraries(gui_listener_exec 
    PUBLIC dependency_library
)

install(TARGETS gui_listener_exec
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()