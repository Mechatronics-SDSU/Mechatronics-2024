#ifndef DESIRED_STATE_NODE_H
#define DESIRED_STATE_NODE_H

#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class DesiredStateNode : public Component
{
    public:
        DesiredStateNode();
        Interface::get_desired_state_service_t createGetDesiredStateService();
        Interface::change_desired_state_service_t createChangeDesiredStateService();
    private:
        Interface::RobotState desired_state;
        Interface::get_desired_state_service_t get_desired_state_service;
        Interface::change_desired_state_service_t change_desired_state_service;
};

#endif
