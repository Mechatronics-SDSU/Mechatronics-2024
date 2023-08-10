#pragma once
#include "rclcpp/rclcpp.hpp"
#include "robot.hpp"
#include <memory>

class Component : public rclcpp::Node
{
    public:
        Component();
    protected:
        virtual void update() = 0;
        void enable();
        void disable();
        void validate();
        bool enabled = false;
        bool valid = false;
};