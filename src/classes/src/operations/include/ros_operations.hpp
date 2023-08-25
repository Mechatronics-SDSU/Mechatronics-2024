#ifndef ROS_OPERATIONS_H
#define ROS_OPERATIONS_H
#include "robot_interface.hpp"

namespace rosOperations
{
    Interface::RobotState copyRobotState(const scion_types::msg::State::SharedPtr msg, Interface::RobotState& robot_state);
    scion_types::msg::State copyRobotState(const Interface::RobotState& robot_state, scion_types::msg::State& msg);
    scion_types::srv::GetDesiredState::Response copyRobotState(const Interface::RobotState& robot_state, scion_types::srv::GetDesiredState::Response& response);
    Interface::RobotState copyRobotState(const Interface::change_desired_state_request_t request, Interface::RobotState& robot_state);
    scion_types::srv::ChangeDesiredState::Response copyRobotState(const Interface::RobotState& robot_state, scion_types::srv::ChangeDesiredState::Response& response);
    Interface::RobotState copyRobotState(const scion_types::srv::GetDesiredState::Response response, Interface::RobotState& robot_state);
    Interface::RobotState copyRobotState(const scion_types::srv::ChangeDesiredState::Response response, Interface::RobotState& robot_state);
    scion_types::srv::GetDesiredState::Response getDesiredState(Interface::get_desired_state_client_t& client, std::string requester_name);
}

#endif