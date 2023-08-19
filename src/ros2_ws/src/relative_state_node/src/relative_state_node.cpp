#include "relative_state_node.hpp"

RelativeStateNode::RelativeStateNode() : Component("relative_state_node")
{
    relative_state_sub = this->create_subscription<std_msgs::msg::String>("relative_state_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    });
}
