#include "robot.hpp"
#include "robot_factory.hpp"
#include "mainwindow.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <iostream>
#include <signal.h>

/* CONFIG_PATH defined in CMakeLists.txt*/

int main(int argc, char** argv)
{
    /* Init Config*/
    rclcpp::init(argc, argv);
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_PATH);
    nlohmann::json json_string = config->getJsonString();

    /* Construct Robot*/
    RobotType type = RobotFactory::getType(json_string["robot"]);
    std::shared_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);

    /* Async Run Read-Only Nodes*/

    /* Run Loop For Write Thread*/
    rclcpp::spin(robot);
    rclcpp::shutdown();   
    return EXIT_SUCCESS;
}