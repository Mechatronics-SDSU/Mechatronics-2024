#include "absolute_state_node.hpp"
#include "state_copy_helper.hpp"
/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

namespace
{
    Interface::state_sub_t createStateSubscription(Interface::node_t& node, std::string topic_name, Interface::RobotState& robot_state)
    {
        return node->create_subscription<scion_types::msg::State>(topic_name, 1, 
        [node, &robot_state](const scion_types::msg::State::SharedPtr msg)
        {
            copyRobotState(msg, robot_state);
        });
    }
}

AbsoluteStateNode::AbsoluteStateNode() : Component("absolute_state_node")
{
    absolute_state_pub =    this->create_publisher<scion_types::msg::State>("absolute_state_data", 10);
    a50_state_sub =         createStateSubscription(this, "a50_state_data",     this->absolute_robot_state);
    ahrs_state_sub =        createStateSubscription(this, "ahrs_state_data",    this->absolute_robot_state);
    zed_pos_state_sub =     createStateSubscription(this, "zed_pos_state_data", this->absolute_robot_state);
}

void AbsoluteStateNode::publishAbsoluteState()
{
    scion_types::msg::State absolute_state = scion_types::msg::State();
    copyRobotState(this->absolute_state, absolute_state);
    this->absolute_state_pub->publish(absolute_state);
}

