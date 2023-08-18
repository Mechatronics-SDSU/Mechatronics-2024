#include "robot.hpp"
#include "robot_factory.hpp"
#include "components.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
std::shared_ptr<Robot> initRobot(const std::string& configuration_path)
{
    return RobotFactory::createRobot(*std::make_unique<Configuration>(configuration_path));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Robot> robot = initRobot(CONFIG_PATH);  // defined in CMakeLists.txt
    std::shared_ptr<std::vector<std::shared_ptr<Component>>> components = Components::CreateComponentVector(*robot);
    rclcpp::spin(robot);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}