#include "brain_node.hpp"

BrainNode::BrainNode() : Component("brain_node")
{
    brain_sub = this->create_subscription<std_msgs::msg::String>("brain_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    });
}
