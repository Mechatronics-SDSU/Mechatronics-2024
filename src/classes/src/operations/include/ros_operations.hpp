#ifndef ROS_OPERATIONS_H
#define ROS_OPERATIONS_H
#include "robot_interface.hpp"

namespace rosOperations
{
    Interface::RobotState copyRobotState(const scion_types::msg::State::SharedPtr msg, Interface::RobotState& robot_state);
    scion_types::msg::State copyRobotState(const Interface::RobotState& robot_state, scion_types::msg::State msg);
}

#endif