#include "robot.hpp"
#include "junebug.hpp"
#include "percy.hpp"
#include <iostream>

Robot::Robot() : Node("Robot")
{
    update_timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Robot::main_update_loop, this));
}

void Robot::main_update_loop()
{
    std::cout << this->name;
}
