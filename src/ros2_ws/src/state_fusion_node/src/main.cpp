#include "state_fusion_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateFusionNode>());
    rclcpp::shutdown();
    return 0;
}
