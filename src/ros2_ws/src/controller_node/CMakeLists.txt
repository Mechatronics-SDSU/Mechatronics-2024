cmake_minimum_required(VERSION 3.5)
project(controller_node)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

add_definitions(-DCONFIG_PATH="${SRC_DIR}/robot/config.json")

set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(${ROOT_DIR}/ros2_ws/src/robot_library ${CMAKE_BINARY_DIR}/robot-build)

# Add your package's source files
add_executable(
    controller_exec 
    ${SRC_DIR}/controller_node/src/controller_node.cpp
    ${SRC_DIR}/controller_node/src/main.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(controller_exec
  robot_library
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    controller_exec PUBLIC 
    ${SRC_DIR}/controller_node/include
)

ament_target_dependencies(controller_exec scion_types rclcpp rclcpp_action std_srvs std_msgs sensor_msgs)

# Install the executable
install(TARGETS controller_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
