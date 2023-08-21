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

scion_types::srv::GetDesiredState::Response rosOperations::copyRobotState(const Interface::RobotState& robot_state, scion_types::srv::GetDesiredState::Response& response)
{
    response.desired_state.orientation.yaw.value =    robot_state.orientation.yaw;
    response.desired_state.orientation.pitch.value =  robot_state.orientation.pitch;
    response.desired_state.orientation.roll.value =   robot_state.orientation.roll;
    response.desired_state.position.x_pos.value =     robot_state.position.x_pos;
    response.desired_state.position.y_pos.value =     robot_state.position.y_pos;
    response.desired_state.position.z_pos.value =     robot_state.position.z_pos;
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

scion_types::srv::ChangeDesiredState::Response rosOperations::copyRobotState(const Interface::RobotState& robot_state, scion_types::srv::ChangeDesiredState::Response& response)
{
    response.desired_state.orientation.yaw.value =    robot_state.orientation.yaw;
    response.desired_state.orientation.pitch.value =  robot_state.orientation.pitch;
    response.desired_state.orientation.roll.value =   robot_state.orientation.roll;
    response.desired_state.position.x_pos.value =     robot_state.position.x_pos;
    response.desired_state.position.y_pos.value =     robot_state.position.y_pos;
    response.desired_state.position.z_pos.value =     robot_state.position.z_pos;
    return response;
}

Interface::RobotState rosOperations::copyRobotState(const scion_types::srv::GetDesiredState::Response response, Interface::RobotState& robot_state)
{
    if (response.desired_state.orientation.yaw.set)   {robot_state.orientation.yaw = response.desired_state.orientation.yaw.value;}
    if (response.desired_state.orientation.pitch.set) {robot_state.orientation.pitch = response.desired_state.orientation.pitch.value;}
    if (response.desired_state.orientation.roll.set)  {robot_state.orientation.roll = response.desired_state.orientation.roll.value;}
    if (response.desired_state.position.x_pos.set)    {robot_state.position.x_pos = response.desired_state.position.x_pos.value;}
    if (response.desired_state.position.y_pos.set)    {robot_state.position.y_pos = response.desired_state.position.y_pos.value;}
    if (response.desired_state.position.z_pos.set)    {robot_state.position.z_pos = response.desired_state.position.z_pos.value;}
    return robot_state;
}

Interface::RobotState rosOperations::copyRobotState(const scion_types::srv::ChangeDesiredState::Response response, Interface::RobotState& robot_state)
{
    if (response.desired_state.orientation.yaw.set)   {robot_state.orientation.yaw = response.desired_state.orientation.yaw.value;}
    if (response.desired_state.orientation.pitch.set) {robot_state.orientation.pitch = response.desired_state.orientation.pitch.value;}
    if (response.desired_state.orientation.roll.set)  {robot_state.orientation.roll = response.desired_state.orientation.roll.value;}
    if (response.desired_state.position.x_pos.set)    {robot_state.position.x_pos = response.desired_state.position.x_pos.value;}
    if (response.desired_state.position.y_pos.set)    {robot_state.position.y_pos = response.desired_state.position.y_pos.value;}
    if (response.desired_state.position.z_pos.set)    {robot_state.position.z_pos = response.desired_state.position.z_pos.value;}
    return robot_state;
}

scion_types::srv::GetDesiredState::Response rosOperations::getDesiredState(rclcpp::Node* node, std::string requester_name)
{
    auto response_received_callback = [&](rclcpp::Client<scion_types::srv::GetDesiredState>::SharedFuture future) {
        auto response = future.get();
        return *response; 
    };
    Interface::get_desired_state_client_t get_desired_state_node_client =  node->create_client<scion_types::srv::GetDesiredState>("get_desired_state");
    auto get_desired_state_request = std::make_shared<scion_types::srv::GetDesiredState::Request>();
    get_desired_state_request->requester_name = requester_name;
    auto future = get_desired_state_node_client->async_send_request(get_desired_state_request, response_received_callback);
    return scion_types::srv::GetDesiredState::Response();
}