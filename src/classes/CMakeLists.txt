cmake_minimum_required(VERSION 3.5)
project(classes)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_target_dependencies(nodes scion_types rclcpp rclcpp_action std_srvs std_msgs sensor_msgs)

add_library(classes
    filters/src/finite_impulse_filter.cpp
    pid_controller/src/pid_controller.cpp
    pid_controller/src/scion_pid_controller.cpp
    pid_controller/src/pid_params.cpp
)

target_include_directories(
    classes
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    pid_controller/include
    operations/include
    can_interface/include
    filters/include
)

ament_package()