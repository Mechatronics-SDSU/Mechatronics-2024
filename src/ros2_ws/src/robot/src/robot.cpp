#include "robot.hpp"
#include "junebug.hpp"
#include "percy.hpp"

std::unordered_map<std::string, RobotType> RobotFactory::robot_types;
std::unordered_map<RobotType, std::function<std::unique_ptr<Robot>(const Configuration&)>> RobotFactory::constructors;

Robot::Robot()
{
    
}

void Robot::main_update_loop()
{
    
}

void RobotFactory::RegisterRobotType(std::string type_name, RobotType type, RobotConstructor constructor) 
{
    robot_types[type_name] = type;
    constructors[type] = constructor;
}

void RobotFactory::registerRobots()
{
    RobotFactory::RegisterRobotType("percy", RobotType::Percy, &Percy::CreatePercy);
    RobotFactory::RegisterRobotType("junebug", RobotType::Junebug, &Junebug::CreateJunebug);    
}

RobotType RobotFactory::getType(std::string type_name)
{
    auto it = robot_types.find(type_name);
     if (it != robot_types.end()) {
        return it->second;
    }
    return RobotType::Percy;
}

std::unique_ptr<Robot> RobotFactory::CreateRobot(RobotType type, const Configuration& config)
{
    auto it = constructors.find(type);
    if (it != constructors.end()) {
        return it->second(config);
    }
    return NULL;
}