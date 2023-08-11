cmake_minimum_required(VERSION 3.5)
project(dependency_library)

set(CMAKE_CXX_STANDARD 20)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(CLASS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${CLASS_DIR}" DIRECTORY)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dependency_library
    ${SRC_DIR}/can_interface/src/can_interface.cpp
    ${SRC_DIR}/filters/src/finite_impulse_filter.cpp
    ${SRC_DIR}/pid_controller/src/pid_controller.cpp
    ${SRC_DIR}/pid_controller/src/scion_pid_controller.cpp
    ${SRC_DIR}/pid_controller/src/pid_params.cpp
    ${SRC_DIR}/operations/src/math_operations.cpp
)

target_include_directories(
    dependency_library
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    ${SRC_DIR}/robot_interface/include
    ${SRC_DIR}/pid_controller/include
    ${SRC_DIR}/operations/include
    ${SRC_DIR}/can_interface/include
    ${SRC_DIR}/filters/include
)

ament_target_dependencies(dependency_library scion_types rclcpp rclcpp_action std_srvs std_msgs sensor_msgs)

install(TARGETS dependency_library
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()