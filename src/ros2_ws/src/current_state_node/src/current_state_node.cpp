#include "current_state_node.hpp"
/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

namespace
{
    void copyRobotState(const scion_types::msg::State::SharedPtr msg, Interface::RobotState& robot_state)
    {

    }

    void copyRobotState(const Interface::RobotState& robot_state, scion_types::msg::State msg)
    {

    }
}

namespace
{
    Interface::state_sub_t createStateSubscription(Interface::node_t node, std::string topic_name, Interface::RobotState& robot_state)
    {
        return node->create_subscription<scion_types::msg::State>(topic_name, 1, 
        [node, &robot_state](const scion_types::msg::State::SharedPtr msg)
        {
            copyRobotState(msg, robot_state);
        });
    }
}
CurrentStateNode::CurrentStateNode() : Component("current_state_node")
{
    absolute_state_pub =    this->create_publisher<scion_types::msg::State>("absolute_current_state_data", 10);
    ahrs_state_sub =        createStateSubscription(this, "ahrs_state_data",    this->absolute_robot_state);
    a50_state_sub =         createStateSubscription(this, "a50_state_data",     this->absolute_robot_state);
    zed_pos_state_sub =     createStateSubscription(this, "zed_pos_state_data", this->absolute_robot_state);
}

void CurrentStateNode::publishAbsoluteState()
{
    scion_types::msg::State absolute_state = scion_types::msg::State();
    this->absolute_state_pub->publish(absolute_state);
}

