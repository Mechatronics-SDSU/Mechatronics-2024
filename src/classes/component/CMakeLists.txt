cmake_minimum_required(VERSION 3.5)
project(component)

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

# Add your package's source files
add_executable(
    component 
    ${SRC_DIR}/component/src/component.cpp
    ${SRC_DIR}/component/src/main.cpp
)

target_include_directories(
    component PUBLIC 
    ${SRC_DIR}/component/include
)

# Install the executable
install(TARGETS component
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 
