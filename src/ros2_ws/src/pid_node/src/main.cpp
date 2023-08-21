#include "pid_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidNode>());
    rclcpp::shutdown();
    return 0;
}
