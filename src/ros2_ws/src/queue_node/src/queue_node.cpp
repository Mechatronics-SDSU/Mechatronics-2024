#include "queue_node.hpp"

QueueNode::QueueNode() : Component("queue_node")
{
    queue_sub = this->create_subscription<std_msgs::msg::String>("queue_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    });
}
