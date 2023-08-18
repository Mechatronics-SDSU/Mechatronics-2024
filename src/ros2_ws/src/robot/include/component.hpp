#pragma once
#include "rclcpp/rclcpp.hpp"
#include <memory>

class Component : public rclcpp::Node
{
    public:
        Component(std::string name);
    protected:
        void enable();
        void disable();
        void validate();
        bool enabled = false;
        bool valid = false;
    private:
        int update_rate;
};