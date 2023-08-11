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
#include "math_operations.hpp"

#define MAX_POWER 100

/* Require:
 * Subscription to ROS built in Joy node
 * CAN Mailbox for CAN Requests
 * Thrust_mapper matrix to take the 6 axis yaw, pitch, roll, x, y, z, and map them to physical movements of
 * the robot that can be implemented using 8 motor thrust values from -100 to 100 
 */
Controller::Controller(Interface::matrix_t thrust_mapper, int motor_count, std::shared_ptr<CanInterface::CanClient> canClient) : Component("controller")
{
    controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>
    ("/joy", 10, std::bind(&Controller::controller_subscription_callback, this, std::placeholders::_1));

    /* x_button, o_button, tri_button, square_button*/
    buttons_ = std::vector<bool>{false, false, false, false};
    button_functions_ = std::vector<button_function> 
    {
        &CanInterface::CanClient::killRobot, 
        &CanInterface::CanClient::allClear, 
        &CanInterface::CanClient::turnOnLight,
        &CanInterface::CanClient::turnOffLight
    };

    this->can_client = can_client;
    this->motor_count = motor_count;
    this->thrust_mapper = thrust_mapper;
}

void Controller::controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
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

    bool x_button = msg->buttons[0];
    bool o_button = msg->buttons[1];
    bool tri_button = msg->buttons[2];
    bool square_button = msg->buttons[3];

    std::vector<float> ctrl_vals = std::vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
    processStickInputs(ctrl_vals);

    std::vector<bool> button_vals{x_button, o_button, tri_button, square_button};
    processButtonInputs(button_vals);
}

// Side effect: motor request equal to power provided by stick
void Controller::processStickInputs(std::vector<float>& ctrl_vals)
{
    ctrl_vals = mathOperations::normalizeCtrlVals(ctrl_vals);
    std::vector<float> thrust_vals = this->thrust_mapper * ctrl_vals;
    can_client->make_motor_request(thrust_vals, this->motor_count, MAX_POWER);
}

// Side effect: CAN request equal to function mapped to button
void Controller::processButtonInputs(std::vector<bool>& button_inputs)
{
    for (int i = 0; i < button_inputs.size(); i++) {
        if (button_inputs[i] && !this->buttons_[i]) {
            this->buttons_[i] = 1;
            // (this->can_client->*(button_functions_[i]))();
        }
        if (!button_inputs[i] && this->buttons_[i]) {
            this->buttons_[i] = 0;
        }
    }
}