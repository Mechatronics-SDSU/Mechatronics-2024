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
        PidNode(const Robot& robot);
    protected:
        Interface::state_sub_t current_state_sub;
        Scion_Position_PID_Controller controller;
        PID_Params pid_params_object;                      // Passed to controller for tuning
        const Robot& robot;
};

#endif
