cmake_minimum_required(VERSION 3.5)
project(can_listener)

get_filename_component(SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${SCRIPTS_DIR}" DIRECTORY)

set(CMAKE_CXX_STANDARD 20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(${ROOT_DIR}/ros2_ws/src/robot_library robot_library_binary/)

# Add your package's source files
add_executable(
    can_listener 
    ${SCRIPTS_DIR}/can_listener/src/simple_can_listener.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(can_listener
  robot_library
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(can_listener scion_types rclcpp rclcpp_action std_srvs std_msgs)

# Install the executable
install(TARGETS can_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
