cmake_minimum_required(VERSION 3.5)

project(gui_node LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DSTYLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/styles/Darkeum.qss")

find_package(Qt5 COMPONENTS Widgets REQUIRED)
# find_package(Qt<Qt5> COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(std_msgs REQUIRED)
# find_library(CORE_DEPENDENCIES core_dependencies)

include_directories(include)

set(UI_FILES
    ui/mainwindow.ui
    ui/mission_planning.ui
    ui/pid_controller.ui
    ui/launch_edit.ui
)

QT5_WRAP_UI(UI_HEADERS ${UI_FILES})

add_library(gui_node STATIC
  src/mainwindow.cpp
  src/mission_planning.cpp
  src/pid_controller.cpp
  src/launch_edit.cpp
  include/mainwindow.hpp
  include/pid_controller.hpp
  include/mission_planning.hpp
  include/launch_edit.hpp
  include/mainwindow.h
  include/mission_planning.h
  include/pid_controller.h
  include/launch_edit.h
  ${UI_HEADERS}
)

# target_include_directories(gui_node PUBLIC
#     ${Qt5Widgets_INCLUDE_DIRS}
#     include
# )

# target_link_libraries(gui_node 
#     PRIVATE Qt5::Widgets
#     ${CORE_DEPENDENCIES}  
# )

add_executable(gui_exec
  src/main.cpp
  src/mainwindow.cpp
  src/mission_planning.cpp
  src/pid_controller.cpp
  src/launch_edit.cpp
  include/mainwindow.hpp
  include/pid_controller.hpp
  include/mission_planning.hpp
  include/launch_edit.hpp
  ${UI_HEADERS}
)

target_include_directories(gui_exec PUBLIC include
                                    ../control_interface/include)

# ament_target_dependencies(gui_node PUBLIC rclcpp scion_types std_msgs)
ament_target_dependencies(gui_exec PUBLIC rclcpp scion_types std_msgs)


target_link_libraries(gui_exec PRIVATE Qt5::Widgets)

install(TARGETS gui_exec
    DESTINATION lib/${PROJECT_NAME}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

ament_package()