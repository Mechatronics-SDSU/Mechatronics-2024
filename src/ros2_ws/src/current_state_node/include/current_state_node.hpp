#ifndef CURRENT_STATE_NODE_H
#define CURRENT_STATE_NODE_H

#include <memory>
#include <vector>
#include <unistd.h>
#include <string>

#include "vector_operations.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"
#include "component.hpp"

/* Subscribe to all relevant sensor information and consolidate it for the other nodes to subscribe to */
class CurrentStateNode : public Component
{
  public:
      CurrentStateNode();
      void publishAbsoluteState();
  protected:
      Interface::RobotState               absolute_robot_state;
      Interface::state_pub_t              absolute_state_pub;
      Interface::state_sub_t              ahrs_state_sub;
      Interface::state_sub_t              a50_state_sub;
      Interface::state_sub_t              zed_pos_state_sub;
};

#endif
