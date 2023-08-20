#include "sub_state_listener_node.hpp"

SubStateListenerNode::SubStateListenerNode() : Component("sub_state_listener_node")
{
    submarine_state_sub = this->create_subscription<scion_types::msg::SubState>("submarine_state", 10, [this](const scion_types::msg::SubState::SharedPtr msg)
    {
        if (msg->host_mode == 0) {
            fprintf(stderr, "\nRobot Killed at %s at line # %d\n", __FILE__,__LINE__);
            exit(EXIT_SUCCESS);
        }
    });
}
