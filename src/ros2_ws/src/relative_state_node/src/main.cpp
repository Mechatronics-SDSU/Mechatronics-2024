#include "relative_state_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeStateNode>());
    rclcpp::shutdown();
    return 0;
}
