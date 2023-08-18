#include "robot.hpp"

Robot::Robot() : Node("Robot")
{
    update_timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Robot::main_update_loop, this));
    can_client = std::make_shared<CanInterface::CanClient>();
}

void Robot::main_update_loop()
{

}
