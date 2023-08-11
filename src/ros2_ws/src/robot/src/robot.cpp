#include "robot.hpp"
#include "junebug.hpp"
#include "percy.hpp"
#include <iostream>

Robot::Robot() : Node("Robot")
{
    update_timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Robot::main_update_loop, this));
    can_client = std::make_shared<CanInterface::CanClient>();
    gui_listener = std::make_shared<GUI_Listener>();
    controller_node = std::make_shared<Controller>(this->getThrustMapper(), this->getMotorCount(), can_client);
}

void Robot::main_update_loop()
{
    rclcpp::spin_some(gui_listener);
}
