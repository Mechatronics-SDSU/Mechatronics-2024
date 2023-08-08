#include "robot.hpp"
#define CONFIG_FILE "/home/mechatronics/clean/src/ros2_ws/src/robot/config.json"

std::unordered_map<std::string, RobotType> RobotFactory::robot_types = {};
std::unordered_map<RobotType, std::function<std::unique_ptr<Robot>(const Configuration&)>> RobotFactory::constructors = {};

int main(int argc, char** argv)
{
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_FILE);
    nlohmann::json json_string = config->getJsonString();

    RobotType type = RobotFactory::getType(json_string["robot"]);
    std::unique_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);
    
    robot->

    return EXIT_SUCCESS;
 }