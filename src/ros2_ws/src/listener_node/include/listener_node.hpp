#ifndef LISTENER_NODE_H
#define LISTENER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"
#include "component.hpp"

class ListenerNode : public Component
{
  public:
      ListenerNode();
  protected:
      Interface::sub_state_sub_t submarine_state_sub;
};

#endif
