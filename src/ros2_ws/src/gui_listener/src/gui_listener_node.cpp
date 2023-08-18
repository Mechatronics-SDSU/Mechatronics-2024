#include "gui_listener_node.hpp"

GUI_Listener::GUI_Listener() : Component("gui_listener")
{
    gui_subscriber = this->create_subscription<std_msgs::msg::String>("gui_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    });
}