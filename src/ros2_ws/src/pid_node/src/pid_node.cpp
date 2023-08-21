#include "pid_node.hpp"
#include "ros_operations.hpp"

namespace
{
    Interface::RobotState getDesiredState()
    {
        scion_types::srv::GetDesiredState::Response response = rosOperations::getDesiredState("pid_node", "pid_get_desired_state_client");
        Interface::RobotState desired_state;
        return rosOperations::copyRobotState(response, desired_state);
    }
}

namespace
{
    Interface::RobotState getCurrentState(const scion_types::msg::State::SharedPtr msg)
    {
        Interface::RobotState current_state;
        return rosOperations::copyRobotState(msg, current_state);
    }  
}

namespace
{
    vector<float> convertFromStateToVector(const Interface::RobotState& state)
    {
        return vector<float>
        {
            state.orientation.yaw, state.orientation.pitch, state.orientation.roll, 
            state.position.x_pos,  state.position.y_pos,    state.position.z_pos
        };
    }
}

namespace
{
    vector<float> getErrors(const vector<float>& current_state, const vector<float>& desired_state) 
    {
        return desired_state - current_state;
    }    
}

PidNode::PidNode() : Component("pid_node")
{
    controller = Scion_Position_PID_Controller(pid_params_object.get_pid_params());
    current_state_sub = this->create_subscription<scion_types::msg::State>("pid_data", 10, [this](const scion_types::msg::State::SharedPtr msg)
    {
        std::vector<float> errors = getErrors(convertFromStateToVector(getCurrentState(msg)), convertFromStateToVector(getDesiredState()));
        std::vector<float> ctrl_vals = controller.update(errors, .10);
    });
}




// vector<float> Controller::getThrusts(vector<float>& current_state, vector<float>& desired_state)
// {
//     vector<float> errors                   {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 
//     vector<float> adjustedErrors           {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
//     vector<float> ctrl_vals                {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 

//     errors = getErrors(current_state,  desired_state);
//     adjustedErrors = adjustErrors(errors);
//     ctrl_vals = this->controller_.update(adjustedErrors, (float)UPDATE_PERIOD_RAW / 1000);
//     return ctrlValsToThrusts(ctrl_vals);
// }    

// vector<float> Controller::ctrlValsToThrusts(vector<float>& ctrl_vals)
// {
//     return this->thrust_mapper_ * ctrl_vals;
// }