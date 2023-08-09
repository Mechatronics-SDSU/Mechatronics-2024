#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "vector_operations.hpp"
#include "control_interface.hpp"

class Controller : public rclcpp::Node
{
    typedef void (Controller::*button_function)();

    public:
        Controller(unique_ptr<Robot> robot);

    private:
        std::vector<bool>                                           buttons_;
        std::vector<button_function>                                button_functions_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      controller_sub_;
    
        void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void processButtonInputs(std::vector<bool>& button_inputs);
        std::vector<float> normalizeCtrlVals(std::vector<float>& ctrl_vals);
};

#endif