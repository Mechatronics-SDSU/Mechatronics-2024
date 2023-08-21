#include "desired_state_node.hpp"
#include "ros_operations.hpp"

Interface::get_desired_state_service_t DesiredStateNode::createGetDesiredStateService()
{
    return this->create_service<scion_types::srv::GetDesiredState>("get_desired_state", [this]
    (Interface::get_desired_state_request_t request, Interface::get_desired_state_response_t response)
    {
        rosOperations::copyRobotState(this->desired_state, *response);
    });
}

Interface::change_desired_state_service_t DesiredStateNode::createChangeDesiredStateService()
{
    return this->create_service<scion_types::srv::ChangeDesiredState>("change_desired_state", [this]
    (Interface::change_desired_state_request_t request, Interface::change_desired_state_response_t response)
    {
        RCLCPP_INFO(this->get_logger(), "%s requested to change desired state", request->requester_name.c_str());
        this->desired_state = rosOperations::copyRobotState(request, this->desired_state);
        rosOperations::copyRobotState(this->desired_state, *response);
    });
}

DesiredStateNode::DesiredStateNode() : Component("desired_state_node")
{
    get_desired_state_service = createGetDesiredStateService();
    change_desired_state_service = createChangeDesiredStateService();
}