#include "state_fusion_node.hpp"

StateFusionNode::StateFusionNode() : Component("state_fusion_node")
{
    state_fusion_sub = this->create_subscription<std_msgs::msg::String>("state_fusion_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    });
}
