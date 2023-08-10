#include "gui_listener.hpp"

GUI_Listener::GUI_Listener() : Node("gui_listener")
{
    gui_subscriber = this->create_subscription<std_msgs::msg::String>("gui_data", 10, [this](const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(msg->data);
    });
}