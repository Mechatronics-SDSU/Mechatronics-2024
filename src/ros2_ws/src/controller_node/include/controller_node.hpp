#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdlib.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "vector_operations.hpp"
#include "robot_interface.hpp"
#include "can_interface.hpp"
#include "component.hpp"

class Controller : public Component
{
    typedef void (CanInterface::CanClient::*button_function)();

    public:
        Controller(Interface::matrix_t thrust_mapper, int motor_count, std::shared_ptr<CanInterface::CanClient> canClient);
    private:
        std::vector<bool>                                           buttons_;
        std::vector<button_function>                                button_functions_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      controller_sub_;
        std::shared_ptr<CanInterface::CanClient>                    can_client;
        Interface::matrix_t                                         thrust_mapper;
        int                                                         motor_count;
    
        void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void processButtonInputs(std::vector<bool>& button_inputs);
};

#endif