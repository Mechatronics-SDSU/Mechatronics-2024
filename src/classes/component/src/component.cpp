#include "component.hpp"
#include <cstring>

Component::Component(std::string name) : Node(name) 
{
    state = std::make_unique<InactiveState>();
}

void Component::spin()
{
    rclcpp::spin_some(shared_from_this());
}

void Component::update(Component& component)
{
    state->update(component);
}

void Component::idle()
{
    return;
}

void ActiveState::update(Component& component) 
{
    component.spin();
}

void InactiveState::update(Component& component) 
{
    component.idle();
}
