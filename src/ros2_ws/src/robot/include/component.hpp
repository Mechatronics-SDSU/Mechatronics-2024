#pragma once
#include "rclcpp/rclcpp.hpp"
#include <memory>

class Component : public rclcpp::Node
{
    public:
        Component(std::string name);
        virtual ~Component() {};
    protected:
        virtual void update();
        bool valid;
    private:
        int update_rate;
};