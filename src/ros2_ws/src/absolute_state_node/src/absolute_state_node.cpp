#include "absolute_state_node.hpp"
#include "ros_operations.hpp"

Interface::state_sub_t AbsoluteStateNode::createStateSubscription(AbsoluteStateNode* node, std::string topic_name, Interface::RobotState& robot_state)
{
    return node->create_subscription<scion_types::msg::State>(topic_name, 1, 
    [this, &node, &robot_state](const scion_types::msg::State::SharedPtr msg)
    {
        publishAbsoluteState(rosOperations::copyRobotState(msg, robot_state));
    });
}

AbsoluteStateNode::AbsoluteStateNode() : Component("absolute_state_node")
{
    absolute_state_pub =    this->create_publisher<scion_types::msg::State>("absolute_state_data", 10);
    a50_state_sub =         createStateSubscription(this, "a50_state_data",     this->absolute_robot_state);
    ahrs_state_sub =        createStateSubscription(this, "ahrs_state_data",    this->absolute_robot_state);
    zed_pos_state_sub =     createStateSubscription(this, "zed_pos_state_data", this->absolute_robot_state);
}

void AbsoluteStateNode::publishAbsoluteState(const Interface::RobotState& robot_state)
{
    scion_types::msg::State absolute_state_msg = scion_types::msg::State();
    rosOperations::copyRobotState(robot_state, absolute_state_msg);
    this->absolute_state_pub->publish(absolute_state_msg);
}
