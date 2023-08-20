cmake_minimum_required(VERSION 3.5)
project(test_can_interface)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(TEST_DIR "${ROS_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${TEST_DIR}" DIRECTORY)

set(CMAKE_CXX_STANDARD 20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# Add gtest library
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# add_subdirectory(${SRC_DIR}/googletest googletest_binary/)
add_subdirectory(${ROOT_DIR}/src/ros2_ws/src/robot_library robot_library_binary/)

# Add your package's source files
add_executable(
    test_can_interface 
    ${SRC_DIR}/test_can_interface/src/test_can_interface.cpp
    ${SRC_DIR}/test_can_interface/src/main.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(test_can_interface
  robot_library
  gtest
  gmock
  gtest_main
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    test_can_interface PUBLIC 
    ${SRC_DIR}/test_can_interface/include
)

ament_target_dependencies(test_can_interface scion_types rclcpp rclcpp_action std_srvs std_msgs)

# Install the executable
install(TARGETS test_can_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
