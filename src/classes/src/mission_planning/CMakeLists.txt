cmake_minimum_required(VERSION 3.5)
project(mission_planning)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

set(CMAKE_CXX_STANDARD 20)

# Add your package's source files
add_executable(
    mission_planning 
    ${SRC_DIR}/mission_planning/src/mission.cpp
    ${SRC_DIR}/mission_planning/src/task.cpp
    ${SRC_DIR}/mission_planning/src/main.cpp
    ${SRC_DIR}/mission_planning/tasks/src/go_through_gate.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(mission_planning
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    mission_planning PUBLIC 
    ${SRC_DIR}/mission_planning/include
    ${SRC_DIR}/mission_planning/tasks/include
)

# Install the executable
install(TARGETS mission_planning
  DESTINATION lib/${PROJECT_NAME}
)
