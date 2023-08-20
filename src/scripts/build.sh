. s

cd ../classes
build scion_types

. s
build dependency_library

cd ../ros2_ws
build robot_library
colcon build

. s
cd ../../test/ros2_ws
build test_library

. s