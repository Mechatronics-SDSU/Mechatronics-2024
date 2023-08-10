#pragma once
#include <memory>
#include <unordered_map>
#include <cstring>
#include <functional>
#include "robot.hpp"
#include "percy.hpp"
#include "junebug.hpp"

enum class RobotType 
{
    Percy,
    Junebug,
    Scion
};

class RobotFactory 
{
    public:
        using RobotConstructor = std::function<std::shared_ptr<Robot>(const Configuration&)>;
        static void RegisterRobotType(std::string type_name, RobotType type, RobotConstructor constructor);
        static void registerRobots();
        static RobotType getType(std::string type_name);
        static std::shared_ptr<Robot> CreateRobot(RobotType type, const Configuration& config);
    private:
        static std::unordered_map<std::string, RobotType> robot_types;
        static std::unordered_map<RobotType, RobotConstructor> constructors;
};


