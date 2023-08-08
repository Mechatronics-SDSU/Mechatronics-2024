
origin_directory=$(pwd)
cd ../../ros2_ws/

. /opt/ros/$ROS_DISTRO/setup.sh

colcon build --packages-select scion_types
. install/setup.sh

# Perform colcon build, ignoring the specified packages
colcon build --packages-ignore "scion_types"

. install/setup.sh

cd $origin_directory

