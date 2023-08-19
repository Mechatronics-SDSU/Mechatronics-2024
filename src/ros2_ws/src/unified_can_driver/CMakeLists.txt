cmake_minimum_required(VERSION 3.5)
project(unified_can_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)

include_directories(include)
include_directories(include/module)
include_directories(${SRC_DIR}/robot/include)


# target executable
add_library(
	unified_can_driver_library
	${SRC_DIR}/robot/src/component.cpp
	src/base_driver/unified_can_driver.cpp
	src/base_driver/can_send_service.cpp
	src/base_driver/can_mailbox_topic.cpp
	src/base_driver/send_command_service.cpp
	src/base_driver/state_decoder.cpp
	src/mbox_can.cpp
	src/module_loader.cpp
# Modules:
	src/module/dvl.cpp
	src/module/ms5837.cpp
	src/module/brlight.cpp
	src/module/brping1.cpp
	src/module/module.cpp
)

add_executable(
	unified_can_driver
	src/base_driver/main.cpp
)

target_link_libraries(
  unified_can_driver
  unified_can_driver_library
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(unified_can_driver_library rclcpp scion_types)

install(TARGETS
	unified_can_driver
	DESTINATION lib/${PROJECT_NAME})

install(TARGETS unified_can_driver_library
  DESTINATION lib/${PROJECT_NAME}
)

# debug
add_compile_options(-g)

ament_package()
