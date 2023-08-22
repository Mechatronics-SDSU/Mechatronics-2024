#include "translator_node.hpp"

TranslatorNode::TranslatorNode() : Component("translator_node")
{
    translator_sub = this->create_subscription<std_msgs::msg::String>("translator_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    });
}
