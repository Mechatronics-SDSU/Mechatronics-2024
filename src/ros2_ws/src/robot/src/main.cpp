#include "robot.hpp"
#include "robot_factory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "async_nodes.hpp"

std::shared_ptr<Robot> initRobot(int argc, char* argv[])
{
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_PATH); // defined in CMakeLists.txt
    nlohmann::json json_string = config->getJsonString();
    RobotType type = RobotFactory::getType(json_string["robot"]);
    return RobotFactory::CreateRobot(type, *config);
}

/* Robot entry point */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Robot> robot = initRobot(argc, argv);
    asyncNodes::startup();
    // rclcpp::spin(robot);
    rclcpp::shutdown();   
    return EXIT_SUCCESS;
}