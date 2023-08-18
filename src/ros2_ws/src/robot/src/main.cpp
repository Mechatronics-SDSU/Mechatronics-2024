#include "robot.hpp"
#include "robot_factory.hpp"
#include "rclcpp/rclcpp.hpp"

std::shared_ptr<Robot> initRobot(const std::string& configuration_path)
{
    return RobotFactory::createRobot(*std::make_unique<Configuration>(configuration_path));
}

/* Robot entry point */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Robot> robot = initRobot(CONFIG_PATH);  // defined in CMakeLists.txt
    rclcpp::spin(robot);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}