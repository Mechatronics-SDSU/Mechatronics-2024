#include "component.hpp"

Component::Component(const std::string& node_name, std::unique_ptr<Robot> robot) : Node(node_name)
{
    robot_parent = robot;
}