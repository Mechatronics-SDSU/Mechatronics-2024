#pragma once
#include <memory>
#include "robot.hpp"

enum class RobotType 
{
    Percy, Junebug, Scion
};

namespace RobotFactory 
{
    std::shared_ptr<Robot> createRobot(const Configuration& config);
}
