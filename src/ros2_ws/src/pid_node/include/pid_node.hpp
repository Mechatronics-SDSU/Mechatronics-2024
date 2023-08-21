#ifndef PID_NODE_H
#define PID_NODE_H

#include "robot_interface.hpp"
#include "scion_pid_controller.hpp"   
#include "pid_params.hpp"
#include "vector_operations.hpp"
#include "robot.hpp"
#include "component.hpp"
#include <memory>

class PidNode : public Component
{
    public:
        PidNode(Robot& robot);
        scion_types::srv::GetDesiredState::Response getDesiredState(std::string requester_name, std::string temp_client_name);
        Interface::RobotState getDesiredState();
    protected:
        Interface::state_sub_t current_state_sub;
        Interface::get_desired_state_client_t get_desired_state_node_client;
        Scion_Position_PID_Controller controller;
        PID_Params pid_params_object;                      // Passed to controller for tuning
        Robot& robot;
};

#endif
