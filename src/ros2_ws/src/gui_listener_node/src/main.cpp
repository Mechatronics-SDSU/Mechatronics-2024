#include "gui_listener_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GUI_Listener>());
  rclcpp::shutdown();
  return 0;
}