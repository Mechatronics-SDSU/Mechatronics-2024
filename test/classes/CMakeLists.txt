cmake_minimum_required(VERSION 3.5)
project(classes_test)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

add_library(classes
    filter/src/filter.cpp
    pid_controller/src/pid_controller.cpp
    pid_controller/src/scion_pid_controller.cpp
    pid_controller/src/pid_params.cpp
)

target_include_directories(
    classes
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    pid_controller/include
    filter/include
)