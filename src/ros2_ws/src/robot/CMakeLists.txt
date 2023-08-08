cmake_minimum_required(VERSION 3.12)  # Adjust as needed
project(robot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Add your source files
add_executable(
    robots
    src/robot.cpp
    src/main.cpp
)

# Add any necessary include directories
target_include_directories(robots PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)