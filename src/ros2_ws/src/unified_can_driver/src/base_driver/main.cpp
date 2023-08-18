#include "unified_can_driver.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UnifiedCanDriver>());
	rclcpp::shutdown();
	return 0;
}