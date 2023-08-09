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

#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>
#include "controller_node.hpp"

#define MAX_POWER 100

using namespace std;
using std::placeholders::_1;

/* Require:
 * Subscription to ROS built in Joy node
 * CAN Mailbox for CAN Requests
 * Thrust_mapper matrix to take the 6 axis yaw, pitch, roll, x, y, z, and map them to physical movements of
 * the robot that can be implemented using 8 motor thrust values from -100 to 100 
 */
Controller::Controller() : Node("controller")
{
    controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>
    ("/joy", 10, std::bind(&Controller::controller_subscription_callback, this, _1));

    /* x_button, o_button, tri_button */
    buttons_ = vector<bool>{false, false, false, false};
    button_functions_ = vector<button_function> 
    {
        &canClient::killRobot, 
        &canClient::allClear, 
        &canClient::turnOnLight,
        &canClient::turnOffLight
    };

    canClient::setBotInSafeMode(can_client_);
}

void Controller::controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    vector<float> axes = msg->axes;
    vector<int> buttons = msg->buttons;

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
    
    /* Multiply our 8 x 6 mapper matrix by our 6 x 1 ctrl_vals to get an 8 x 1 vector of thrust values (a vector with 8 values) */

    vector<float> ctrl_vals = vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
    ctrl_vals = normalizeCtrlVals(ctrl_vals);
    vector<float> thrust_vals = this->thrust_mapper_ * ctrl_vals;

    bool x_button = msg->buttons[0];
    bool o_button = msg->buttons[1];
    bool tri_button = msg->buttons[2];
    bool square_button = msg->buttons[3];
    vector<bool> button_vals{x_button, o_button, tri_button, square_button};

    make_CAN_request(thrust_vals);
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


vector<float> Controller::normalizeCtrlVals(vector<float>& ctrl_vals)
{
    vector<float> normalized{0,0,0,0,0,0};
    
    float vectorTotal = abs(ctrl_vals[0]) + abs(ctrl_vals[3]) + abs(ctrl_vals[4]); // yaw, x, y
    float nonVectorTotal = abs(ctrl_vals[1]) + abs(ctrl_vals[2]) + abs(ctrl_vals[5]); // roll, pitch, z

    normalized[0] = ctrl_vals[0];
    normalized[1] = ctrl_vals[1];
    normalized[2] = ctrl_vals[2];
    normalized[3] = ctrl_vals[3];
    normalized[4] = ctrl_vals[4];
    normalized[5] = ctrl_vals[5];

    if (vectorTotal > 1)
    {
        normalized[0] = ctrl_vals[0] / vectorTotal;
        normalized[3] = ctrl_vals[3] / vectorTotal;
        normalized[4] = ctrl_vals[4] / vectorTotal;
    }
    if (nonVectorTotal > 1)
    {
        normalized[1] = ctrl_vals[1] / nonVectorTotal;
        normalized[2] = ctrl_vals[2] / nonVectorTotal;
        normalized[5] = ctrl_vals[5] / nonVectorTotal;
    }

    if (vectorTotal == 0)
    {
        normalized[0] = 0;
        normalized[3] = 0;
        normalized[4] = 0;
    }

    if (nonVectorTotal == 0)
    {
        normalized[1] = 0;
        normalized[2] = 0;
        normalized[5] = 0;
    }

    return normalized;
}


