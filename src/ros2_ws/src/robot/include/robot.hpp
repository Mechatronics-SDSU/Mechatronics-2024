#pragma once

#include <memory>
#include <unordered_map>
#include <cstring>
#include <functional>
#include "robot_interface.hpp"
#include "configuration.hpp"
#define protected public 

class Robot
{
    public: 
        virtual void main_update_loop();
    protected:
        std::string name;
        int motor_count;
        Interface::matrix_t thrust_mapper;
};

enum class RobotType 
{
    Percy,
    Junebug,
    Scion
};

class RobotFactory 
{
    public:
        using RobotConstructor = std::function<std::unique_ptr<Robot>(const Configuration&)>;
        static void RegisterRobotType(std::string type_name, RobotType type, RobotConstructor constructor);
        static void registerRobots();
        static RobotType getType(std::string type_name);
        static std::unique_ptr<Robot> CreateRobot(RobotType type, const Configuration& config);
    private:
        static std::unordered_map<std::string, RobotType> robot_types;
        static std::unordered_map<RobotType, RobotConstructor> constructors;
};

