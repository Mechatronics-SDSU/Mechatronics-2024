#include "robot.hpp"

void Robot::getName()
{
    std::cout << this->name_;
}

void RobotFactory::RegisterRobotType(RobotType type, RobotConstructor constructor) 
{
    constructors[type] = constructor;
    // constructors.insert(std::make_pair(type, constructor));
}

std::unique_ptr<Robot> RobotFactory::CreateRobot(RobotType type, const Configuration& config)
{
    auto it = constructors.find(type);
    if (it != constructors.end()) {
        return it->second(config);
    }
    return NULL;
}