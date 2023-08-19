#pragma once
#include "rclcpp/rclcpp.hpp"
#include <memory>

class Component;
class ComponentState;
class InactiveState;
class ActiveState;

class ComponentState
{
    public:
        virtual ~ComponentState() {}
        virtual void update(Component& component) = 0;
};

class ActiveState : public ComponentState
{
    virtual void update(Component& component);
};

class InactiveState : public ComponentState
{
    virtual void update(Component& component);
};

class Component : public rclcpp::Node
{
    public:
        Component(std::string name);
        virtual ~Component() {};
        virtual void update(Component& component);
        virtual void spin();
        virtual void idle();
    protected:
        int update_rate;
        std::unique_ptr<ComponentState> state;
};