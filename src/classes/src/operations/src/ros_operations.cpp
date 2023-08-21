#include "ros_operations.hpp"

/* move from left hand side -> right hand side*/
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

Interface::get_desired_state_response_t rosOperations::copyRobotState(const Interface::RobotState& robot_state, const Interface::get_desired_state_response_t& response)
{
    response->desired_state.orientation.yaw.value =    robot_state.orientation.yaw;
    response->desired_state.orientation.pitch.value =  robot_state.orientation.pitch;
    response->desired_state.orientation.roll.value =   robot_state.orientation.roll;
    response->desired_state.position.x_pos.value =     robot_state.position.x_pos;
    response->desired_state.position.y_pos.value =     robot_state.position.y_pos;
    response->desired_state.position.z_pos.value =     robot_state.position.z_pos;
    return response;
}

Interface::RobotState rosOperations::copyRobotState(const Interface::change_desired_state_request_t request, Interface::RobotState& robot_state)
{
    if (request->requested_state.orientation.yaw.set)   {robot_state.orientation.yaw = request->requested_state.orientation.yaw.value;}
    if (request->requested_state.orientation.pitch.set) {robot_state.orientation.pitch = request->requested_state.orientation.pitch.value;}
    if (request->requested_state.orientation.roll.set)  {robot_state.orientation.roll = request->requested_state.orientation.roll.value;}
    if (request->requested_state.position.x_pos.set)    {robot_state.position.x_pos = request->requested_state.position.x_pos.value;}
    if (request->requested_state.position.y_pos.set)    {robot_state.position.y_pos = request->requested_state.position.y_pos.value;}
    if (request->requested_state.position.z_pos.set)    {robot_state.position.z_pos = request->requested_state.position.z_pos.value;}
    return robot_state;
}

Interface::change_desired_state_response_t rosOperations::copyRobotState(const Interface::RobotState& robot_state, const Interface::change_desired_state_response_t& response)
{
    response->desired_state.orientation.yaw.value =    robot_state.orientation.yaw;
    response->desired_state.orientation.pitch.value =  robot_state.orientation.pitch;
    response->desired_state.orientation.roll.value =   robot_state.orientation.roll;
    response->desired_state.position.x_pos.value =     robot_state.position.x_pos;
    response->desired_state.position.y_pos.value =     robot_state.position.y_pos;
    response->desired_state.position.z_pos.value =     robot_state.position.z_pos;
    return response;
}