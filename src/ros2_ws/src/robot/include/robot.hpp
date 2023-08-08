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
        void getName();
    protected:
        std::string name_;
        int motor_count_;
};

using RobotConstructor = std::function<std::unique_ptr<Robot>(const Configuration&)>;

class RobotFactory 
{
public:

    static void RegisterRobotType(RobotType type, RobotConstructor constructor);
    static std::unique_ptr<Robot> CreateRobot(RobotType type, const Configuration& config);
    static std::unordered_map<RobotType, RobotConstructor> constructors;
};

enum class RobotType 
{
    Percy,
    Junebug,
    Scion
};

#endif // ROBOT_H