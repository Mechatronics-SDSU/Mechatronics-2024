#include "async_nodes.hpp"
#include "gui_listener_node.hpp"

void asyncNodes::startup()
{
    std::shared_ptr<GUI_Listener> gui_listener_node = std::make_shared<GUI_Listener>();
    rclcpp::executors::MultiThreadedExecutor spinner;
    spinner.add_node(gui_listener_node);
    spinner.spin();
}