#include "translator_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TranslatorNode>());
    rclcpp::shutdown();
    return 0;
}
