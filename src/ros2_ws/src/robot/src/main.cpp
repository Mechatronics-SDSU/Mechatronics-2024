#include "robot.hpp"
#include "junebug.hpp"
#include "percy.hpp"
#define CONFIG_FILE "/home/mechatronics/clean/src/ros2_ws/src/robot/config.json"

namespace 
{
    void registerRobots()
    {
        RobotFactory::RegisterRobotType(RobotType::Percy, &Percy::CreatePercy);
        RobotFactory::RegisterRobotType(RobotType::Junebug, &Junebug::CreateJunebug);
    }
}

std::unordered_map<RobotType, RobotConstructor> RobotFactory::constructors = {};
int main(int argc, char** argv)
{
    registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_FILE);
    nlohmann::json json_string = config->getJsonString();
    RobotType type = (json_string["robot"] == "percy") ? RobotType::Percy : RobotType::Junebug;
    std::unique_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);
    robot->getName();
    return EXIT_SUCCESS;
 }