#include "controller_node.hpp"
#include "robot_factory.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::unique_ptr<Configuration> configuration = std::make_unique<Configuration>(CONFIG_PATH); // Declared in CMakeLists.txt
  std::shared_ptr<Robot> robot = RobotFactory::createRobot(*configuration);
  rclcpp::spin(std::make_shared<Controller>(*robot));
  rclcpp::shutdown();
  return 0;
}