#include "sub_state_listener_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubStateListenerNode>());
    rclcpp::shutdown();
    return 0;
}
