#ifndef ROS_OPERATIONS_H
#define ROS_OPERATIONS_H
#include "robot_interface.hpp"

namespace rosOperations
{
    Interface::RobotState copyRobotState(const scion_types::msg::State::SharedPtr msg, Interface::RobotState& robot_state);
    scion_types::msg::State copyRobotState(const Interface::RobotState& robot_state, const scion_types::msg::State& msg);
    Interface::get_desired_state_response_t copyRobotState(const Interface::RobotState& robot_state, const Interface::get_desired_state_response_t& response);
    Interface::RobotState copyRobotState(const Interface::change_desired_state_request_t request, Interface::RobotState& robot_state);
    Interface::change_desired_state_response_t copyRobotState(const Interface::RobotState& robot_state, const Interface::change_desired_state_response_t& response);
}

#endif