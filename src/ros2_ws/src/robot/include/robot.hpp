#ifndef ROBOT_H
#define ROBOT_H

#include <memory>
#include <unordered_map>
#include <cstring>
#include <functional>
#include "robot_interface.hpp"
#include "configuration.hpp"

class Robot;
class RobotFactory;
enum class RobotType;

class Robot
{
    public: 
        virtual void getName();
        virtual void main_update_loop();
    protected:
        std::string name;
        int motor_count;
};

class RobotFactory 
{
public:
    using RobotConstructor = std::function<std::unique_ptr<Robot>(const Configuration&)>;
    static void RegisterRobotType(std::string type_name, RobotType type, RobotConstructor constructor);
    static void registerRobots();
    static RobotType getType(std::string type_name);
    static std::unique_ptr<Robot> CreateRobot(RobotType type, const Configuration& config);
    static std::unordered_map<std::string, RobotType> robot_types;
    static std::unordered_map<RobotType, RobotConstructor> constructors;
};

enum class RobotType 
{
    Percy,
    Junebug,
    Scion
};

#endif // ROBOT_H