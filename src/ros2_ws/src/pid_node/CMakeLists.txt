cmake_minimum_required(VERSION 3.12)  # Adjust as needed
project(pid_node)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Add your source files
add_executable(
    pid_exec
    src/robot.cpp
    src/main.cpp
)

# Add any necessary include directories
target_include_directories(pid_exec PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)