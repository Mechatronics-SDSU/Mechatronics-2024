#pragma once
#include "robot_interface.hpp"
// #include "can_interface.hpp"
#include "configuration.hpp"
#define protected public 

class Robot : public rclcpp::Node
{
    public: 
        Robot();
        virtual void main_update_loop();
    protected:
        std::string name;
        int motor_count;
        Interface::matrix_t thrust_mapper;
        Interface::ros_timer_t update_timer;
        // CanInterface::CanClient can_client;
};
