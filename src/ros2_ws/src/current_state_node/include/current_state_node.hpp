#ifndef CURRENT_STATE_H
#define CURRENT_STATE_H

#include <memory>
#include <vector>
#include <unistd.h>
#include <string>

#include "vector_operations.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"
#include "component.hpp"

/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

class CurrentStateNode : public Component
{
  public:
      CurrentStateNode();
  protected:
      Interface::RobotState               absolute_robot_state;
      Interface::RobotState               relative_robot_state;
      Interface::state_pub_t              absolute_state_pub;
      Interface::state_pub_t              relative_state_pub;
      Interface::state_sub_t              ahrs_state_sub;
      Interface::state_sub_t              ahrs_state_sub;
      Interface::state_sub_t              a50_state_sub;
      Interface::state_sub_t              zed_pos_state_sub;
      Interface::ros_trigger_service_t    reset_relative_state_service;
      Interface::ros_trigger_service_t    reset_relative_position_service;
      void publishAbsoluteState();
      void publishRelativeState();
      void resetRelativeState (const Interface::trigger_request_t request, const Interface::trigger_response_t response);
      void resetRelativePosition (const Interface::trigger_request_t request, const Interface::trigger_response_t response);
};

#endif
