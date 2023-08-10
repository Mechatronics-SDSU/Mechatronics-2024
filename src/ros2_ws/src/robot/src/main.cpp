#include "robot.hpp"
#include "mainwindow.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <iostream>
#define CONFIG_FILE "/home/mechatronics/robots/src/ros2_ws/src/robot/config.json"

int main(int argc, char** argv)
{
    /* Init Config*/
    rclcpp::init(argc, argv);
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_FILE);
    nlohmann::json json_string = config->getJsonString();

    /* Construct Robot*/
    RobotType type = RobotFactory::getType(json_string["robot"]);
    std::unique_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);

    /* Async Run Read-Only Nodes*/
    // MainWindow::initiate(argc, argv);
    
    /* Run Loop For Write Thread*/
    robot->main_update_loop();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}