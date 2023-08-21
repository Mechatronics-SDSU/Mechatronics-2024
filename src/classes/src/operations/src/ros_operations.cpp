#include "ros_operations.hpp"

Interface::RobotState rosOperations::copyRobotState(const scion_types::msg::State::SharedPtr msg, Interface::RobotState& robot_state)
{
    if (msg->orientation.yaw.set)   {robot_state.orientation.yaw = msg->orientation.yaw.value;}
    if (msg->orientation.pitch.set) {robot_state.orientation.pitch = msg->orientation.pitch.value;}
    if (msg->orientation.roll.set)  {robot_state.orientation.roll = msg->orientation.roll.value;}
    if (msg->position.x_pos.set)    {robot_state.position.x_pos = msg->position.x_pos.value;}
    if (msg->position.y_pos.set)    {robot_state.position.y_pos = msg->position.y_pos.value;}
    if (msg->position.z_pos.set)    {robot_state.position.z_pos = msg->position.z_pos.value;}
    return robot_state;
}

scion_types::msg::State rosOperations::copyRobotState(const Interface::RobotState& robot_state, scion_types::msg::State& msg)
{
    msg.orientation.yaw.value =    robot_state.orientation.yaw;
    msg.orientation.pitch.value =  robot_state.orientation.pitch;
    msg.orientation.roll.value =   robot_state.orientation.roll;
    msg.position.x_pos.value =     robot_state.position.x_pos;
    msg.position.y_pos.value =     robot_state.position.y_pos;
    msg.position.z_pos.value =     robot_state.position.z_pos;
    return msg;
}
