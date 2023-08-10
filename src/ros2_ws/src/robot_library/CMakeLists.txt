cmake_minimum_required(VERSION 3.5)
project(robot_library)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(STYLESHEET_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/styles.qss")
add_definitions(-DSTYLE_SHEET_FILE_PATH="${SRC_DIR}/gui_node/styles.qss")
add_definitions(-DCONFIG_PATH="${SRC_DIR}/robot/config.json")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(${ROOT_DIR}/classes/src/dependency_library ${CMAKE_BINARY_DIR}/classes-build)

configure_file(${SRC_DIR}/robot/config.json ${CMAKE_BINARY_DIR}/config.json COPYONLY)

# Add your package's source files
add_library(
    robot_library 
    ${SRC_DIR}/robot/src/robot.cpp
    ${SRC_DIR}/robot/src/robot_factory.cpp
    ${SRC_DIR}/robot/src/configuration.cpp
    ${SRC_DIR}/robot/src/junebug.cpp
    ${SRC_DIR}/robot/src/percy.cpp
    ${SRC_DIR}/robot/src/component.cpp
    ${SRC_DIR}/gui_node/src/mainwindow.cpp
    ${SRC_DIR}/gui_node/src/mainwindow.ui
    ${SRC_DIR}/gui_node/include/mainwindow.hpp
    ${SRC_DIR}/gui_listener/src/gui_listener_node.cpp
)

add_executable(
    robot
    ${SRC_DIR}/robot/src/main.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(robot_library
  Qt${QT_VERSION_MAJOR}::Widgets
  dependency_library
  ${rclcpp_LIBRARIES}
)

# Link gtest and other libraries to  executable
target_link_libraries(robot
  Qt${QT_VERSION_MAJOR}::Widgets
  robot_library
  dependency_library
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    robot_library PUBLIC 
    ${SRC_DIR}/robot/include
    ${SRC_DIR}/gui_node/include
    ${SRC_DIR}/gui_listener/include
)

target_include_directories(
    robot PUBLIC 
    ${SRC_DIR}/robot/include
    ${SRC_DIR}/gui_node/include
    ${SRC_DIR}/gui_listener/include
)

ament_target_dependencies(robot_library scion_types rclcpp rclcpp_action std_srvs std_msgs sensor_msgs)
ament_target_dependencies(robot scion_types rclcpp rclcpp_action std_srvs std_msgs sensor_msgs)

# Install the executable
install(TARGETS robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
