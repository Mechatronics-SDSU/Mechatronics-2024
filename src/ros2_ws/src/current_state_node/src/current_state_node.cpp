#include "current_state_node.hpp"
/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

namespace
{
    void copyRobotState(const scion_types::msg::State::SharedPtr msg, Interface::RobotState& robot_state)
    {
        if (msg->orientation.yaw.set)   {robot_state.orientation.yaw = msg->orientation.yaw.value;}
        if (msg->orientation.pitch.set) {robot_state.orientation.pitch = msg->orientation.pitch.value;}
        if (msg->orientation.roll.set)  {robot_state.orientation.roll = msg->orientation.roll.value;}
        if (msg->position.x_pos.set)    {robot_state.position.x_pos = msg->position.x_pos.value;}
        if (msg->position.y_pos.set)    {robot_state.position.y_pos = msg->position.y_pos.value;}
        if (msg->position.z_pos.set)    {robot_state.position.z_pos = msg->position.z_pos.value;}
    }

    void copyRobotState(const Interface::RobotState& robot_state, scion_types::msg::State msg)
    {
        msg.orientation.yaw.value =    robot_state.orientation.yaw;
        msg.orientation.pitch.value =  robot_state.orientation.pitch;
        msg.orientation.roll.value =   robot_state.orientation.roll;
        msg.position.x_pos.value =     robot_state.position.x_pos;
        msg.position.y_pos.value =     robot_state.position.y_pos;
        msg.position.z_pos.value =     robot_state.position.z_pos;
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
    using std::placeholders::_1; 
    using std::placeholders::_2;
    absolute_state_pub =                this->create_publisher<scion_types::msg::State>("absolute_current_state_data", 10);
    relative_state_pub =                this->create_publisher<scion_types::msg::State>("relative_current_state_data", 10);
    reset_relative_state_service =      this->create_service<std_srvs::srv::Trigger>("reset_relative_state", std::bind(&CurrentStateNode::resetRelativeState, this, _1, _2));
    reset_relative_position_service =   this->create_service<std_srvs::srv::Trigger>("reset_relative_position", std::bind(&CurrentStateNode::resetRelativePosition, this, _1, _2));
}

void CurrentStateNode::publishAbsoluteState()
{
    scion_types::msg::State absolute_state = scion_types::msg::State();
    this->absolute_state_pub->publish(absolute_state);
}

void CurrentStateNode::publishRelativeState()
{
    scion_types::msg::State relative_state = scion_types::msg::State();
    this->relative_state_pub->publish(relative_state);
}

void CurrentStateNode::resetRelativeState (const Interface::trigger_request_t request, Interface::trigger_response_t response)
{
    RCLCPP_INFO(this->get_logger(), "Incoming Request to Reset Relative State\n");
}

void CurrentStateNode::resetRelativePosition (const Interface::trigger_request_t request, Interface::trigger_response_t response)
{
    RCLCPP_INFO(this->get_logger(), "Incoming Request to Reset Relative Position\n");
}
