#include "robot.hpp"
#include <filesystem>
#define CONFIG_FILE "/home/mechatronics/robots/src/ros2_ws/src/robot/config.json"

int main(int argc, char** argv)
{
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_FILE);
    nlohmann::json json_string = config->getJsonString();

    RobotType type = RobotFactory::getType(json_string["robot"]);
    std::unique_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);

    return EXIT_SUCCESS;
}