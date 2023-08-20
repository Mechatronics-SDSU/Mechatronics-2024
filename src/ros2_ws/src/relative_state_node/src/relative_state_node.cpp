#include "relative_state_node.hpp"
#include <functional>
using std::placeholders::_1; 
using std::placeholders::_2;

RelativeStateNode::RelativeStateNode() : Component("relative_state_node")
{
    relative_state_pub = this->create_publisher<scion_types::msg::State>("relative_current_state_data", 1);
    absolute_state_sub = this->create_subscription<scion_types::msg::State>("absolute_current_state_data", 1, std::bind(&RelativeStateNode::absoluteStateSubCallback, this, _1));
    reset_relative_state_service = this->create_service<std_srvs::srv::Trigger>("reset_relative_state", std::bind(&RelativeStateNode::resetRelativeState, this, _1, _2));
    reset_relative_position_service = this->create_service<std_srvs::srv::Trigger>("reset_relative_position", std::bind(&RelativeStateNode::resetRelativePosition, this, _1, _2));
}

void RelativeStateNode::publishRelativeState()
{
    scion_types::msg::State relative_state = scion_types::msg::State();
    this->relative_state_pub->publish(relative_state);
}

void RelativeStateNode::resetRelativeState (const Interface::trigger_request_t request, Interface::trigger_response_t response)
{
    RCLCPP_INFO(this->get_logger(), "Incoming Request to Reset Relative State\n");
}

void RelativeStateNode::resetRelativePosition (const Interface::trigger_request_t request, Interface::trigger_response_t response)
{
    RCLCPP_INFO(this->get_logger(), "Incoming Request to Reset Relative Position\n");
}

void RelativeStateNode::absoluteStateSubCallback(const scion_types::msg::State::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Absolute state callback\n");
}
