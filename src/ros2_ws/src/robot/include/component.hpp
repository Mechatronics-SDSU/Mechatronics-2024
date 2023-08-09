#pragma once
#include "rclcpp/rclcpp.hpp"
#include "robot.hpp"
#include <memory>

class Component : public rclcpp::Node
{
    public:
        Component(const std::string& node_name, std::unique_ptr<Robot> robot);
    protected:
        virtual void update() = 0;
        void enable();
        void disable();
        void validate();
        bool enabled = false;
        bool valid = false;
        std::unique_ptr<Robot> robot_parent;
};