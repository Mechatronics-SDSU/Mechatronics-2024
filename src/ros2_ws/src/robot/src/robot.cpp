#include "robot.hpp"

Robot::Robot(const Configuration& configuration) : Node("Robot"), configuration{configuration}
{
    update_timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Robot::main_update_loop, this));
    can_client = std::make_shared<CanInterface::CanClient>();
}

void Robot::connectComponents(std::shared_ptr<std::vector<std::shared_ptr<Component>>> components)
{
    this->components = components;
}

void Robot::main_update_loop()
{
    for (std::shared_ptr<Component> component : *this->components)
    {
        rclcpp::spin_some(component);
    }
}