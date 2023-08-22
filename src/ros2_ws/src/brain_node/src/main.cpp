#include "brain_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}
