#include "pid_node.hpp"
#include "ros_operations.hpp"

namespace
{
    Interface::RobotState getCurrentState(const scion_types::msg::State::SharedPtr msg)
    {
        Interface::RobotState current_state; return rosOperations::copyRobotState(msg, current_state);
    }  
}

namespace
{
    vector<float> convertFromStateToVector(const Interface::RobotState& state)
    {
        return vector<float> {
            state.orientation.yaw, state.orientation.pitch, state.orientation.roll, state.position.x_pos,  state.position.y_pos, state.position.z_pos
        };
    }

    vector<float> getErrors(const vector<float>& current_state, const vector<float>& desired_state) 
    {
        return desired_state - current_state;
    }    

    vector<float> ctrlValsToThrusts(const Interface::matrix_t& thrust_mapper, const vector<float>& ctrl_vals)
    {
        return thrust_mapper * ctrl_vals;
    }
}

namespace
{
    #define UPDATE_RATE 20
    #define MAX_POWER 100
}

PidNode::PidNode(Robot& robot) : Component("pid_node"), robot{robot}
{
    get_desired_state_node_client =  this->create_client<scion_types::srv::GetDesiredState>("get_desired_state");
    controller = Scion_Position_PID_Controller(pid_params_object.get_pid_params());
    current_state_sub = this->create_subscription<scion_types::msg::State>("absolute_state_data", 10, [this, &robot](const scion_types::msg::State::SharedPtr msg)
    {
        robot.getCanClient()->make_motor_request(
            ctrlValsToThrusts(robot.getThrustMapper(), controller.update(
            getErrors(convertFromStateToVector(getCurrentState(msg)), convertFromStateToVector(getDesiredState()))
            , 1/UPDATE_RATE)), robot.getMotorCount(), MAX_POWER);
    });
}

Interface::RobotState PidNode::getDesiredState()
{
    scion_types::srv::GetDesiredState::Response response = this->getDesiredState("pid_node", "pid_get_desired_state_client");
    Interface::RobotState desired_state; return rosOperations::copyRobotState(response, desired_state);
}

scion_types::srv::GetDesiredState::Response PidNode::getDesiredState(std::string requester_name, std::string temp_client_name)
{
    auto response_received_callback = [&](rclcpp::Client<scion_types::srv::GetDesiredState>::SharedFuture future) {
        auto response = future.get();
        return *response; 
    };
    auto get_desired_state_request = std::make_shared<scion_types::srv::GetDesiredState::Request>();
    get_desired_state_request->requester_name = requester_name;
    this->get_desired_state_node_client->async_send_request(get_desired_state_request, response_received_callback);
    return scion_types::srv::GetDesiredState::Response();
}