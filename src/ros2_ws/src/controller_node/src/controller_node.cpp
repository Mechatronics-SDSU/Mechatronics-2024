/*  
 *  @author Conner Sommerfield
 *  For questions - Zix on Discord
 *  Controller node for transmitting controller commands on PS4 controller as PID alternative
 *  Uses code very similar to PID such as the thrust_mapper matrix
 *  
 *  To run make sure you source properly and:
 *  1. enable the bluetooth device ds4drv
 *  2. ros2 run joy joy_node
 *  3. ros2 run controller_node controller_exec
 */

#include "controller_node.hpp"

#define MAX_POWER 100

/* Require:
 * Subscription to ROS built in Joy node
 * CAN Mailbox for CAN Requests
 * Thrust_mapper matrix to take the 6 axis yaw, pitch, roll, x, y, z, and map them to physical movements of
 * the robot that can be implemented using 8 motor thrust values from -100 to 100 
 */
Controller::Controller(const std::string& node_name, std::unique_ptr<Robot> robot) : Component(node_name, robot)
{
    controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>
    ("/joy", 10, std::bind(&Controller::controller_subscription_callback, this, std::placeholders::_1));

    /* x_button, o_button, tri_button, square_button*/
    buttons_ = std::vector<bool>{false, false, false, false};
    button_functions_ = std::vector<button_function> 
    {
        &canClient::killRobot, 
        &canClient::allClear, 
        &canClient::turnOnLight,
        &canClient::turnOffLight
    };
}

void Controller::controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::vector<float> axes = msg->axes;
    std::vector<int> buttons = msg->buttons;

    /* These map to the actual controller sticks and buttons */

    /* This part randomly changed on me before pool test, I have no idea why, it was working and then I had to change
    the buttons. Before I had it as 0 5 2 3 4 1, but for the version on the day before pool test it is working
    as 0 4 3 2 5 1 */
    float left_x        = -1 *  msg->axes[0];           // yaw
    float right_trigger =       msg->axes[5] - 1;       // pitch
    float left_trigger  =       msg->axes[2] - 1;       // roll
    float right_x       = 1 *   msg->axes[4];           // x
    float right_y       = -1 *  msg->axes[3];           // y
    float left_y        = -1 *  msg->axes[1];           // z
    
    /* Multiply our 8 x 6 mapper matrix by our 6 x 1 ctrl_vals to get an 8 x 1 std::vector of thrust values (a std::vector with 8 values) */

    std::vector<float> ctrl_vals = std::vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
    ctrl_vals = normalizeCtrlVals(ctrl_vals);
    std::vector<float> thrust_vals = robot_parent->thrust_mapper * ctrl_vals;

    bool x_button = msg->buttons[0];
    bool o_button = msg->buttons[1];
    bool tri_button = msg->buttons[2];
    bool square_button = msg->buttons[3];
    std::vector<bool> button_vals{x_button, o_button, tri_button, square_button};

    canClient::make_CAN_request(thrust_vals, robot_parent->motor_count, MAX_POWER);
    processButtonInputs(button_vals);
}


// 0: x, 1: o, 2: tri, 3: square
void Controller::processButtonInputs(vector<bool>& button_inputs)
{
    for (int i = 0; i < button_inputs.size(); i++)
    {
        if (button_inputs[i] && !this->buttons_[i]) 
        {
            this->buttons_[i] = 1;
            (this->*(button_functions_[i]))();
        }
        if (!button_inputs[i] && this->buttons_[i]) 
        {
            this->buttons_[i] = 0;
        }
    }
}