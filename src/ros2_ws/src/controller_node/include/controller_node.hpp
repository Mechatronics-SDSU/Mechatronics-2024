#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdlib.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "vector_operations.hpp"
#include "robot.hpp"
#include "component.hpp"

class Controller : public Component
{
    typedef void (CanInterface::CanClient::*button_function)();

    public:
        Controller(const Robot& robot);
    private:
        std::vector<bool>                                           buttons_;
        std::vector<button_function>                                button_functions_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      controller_sub_;
        const Robot& robot;

        void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void processStickInputs(std::vector<float>& ctrl_vals);
        void processButtonInputs(std::vector<bool>& button_inputs);
};

#endif