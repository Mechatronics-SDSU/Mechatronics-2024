cmake_minimum_required(VERSION 3.5)
project(translator_node)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(${ROOT_DIR}/classes/src/dependency_library ${CMAKE_BINARY_DIR}/classes-build)

# Add your package's source files
add_executable(
    translator_exec 
    ${SRC_DIR}/translator_node/src/translator_node.cpp
    ${SRC_DIR}/translator_node/src/main.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(translator_exec
  dependency_library
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    translator_exec PUBLIC 
    ${SRC_DIR}/translator_node/include
)

ament_target_dependencies(translator_exec scion_types rclcpp rclcpp_action std_srvs std_msgs sensor_msgs)

# Install the executable
install(TARGETS translator_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 
