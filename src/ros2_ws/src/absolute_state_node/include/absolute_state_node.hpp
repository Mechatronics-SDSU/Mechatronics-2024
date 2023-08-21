#ifndef ABSOLUTE_STATE_NODE_H
#define ABSOLUTE_STATE_NODE_H

#include <memory>
#include <vector>
#include <unistd.h>
#include <string>

#include "vector_operations.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interface.hpp"
#include "component.hpp"

/* Subscribe to all relevant sensor information and consolidate it for the other nodes to subscribe to */
class AbsoluteStateNode : public Component
{
    public:
        AbsoluteStateNode();
        void publishAbsoluteState(const Interface::RobotState& robot_state);
        Interface::state_sub_t createStateSubscription(AbsoluteStateNode* node, std::string topic_name, Interface::RobotState& robot_state);
    protected:
        Interface::RobotState               absolute_robot_state;
        Interface::state_pub_t              absolute_state_pub;
        Interface::state_sub_t              ahrs_state_sub;
        Interface::state_sub_t              a50_state_sub;
        Interface::state_sub_t              zed_pos_state_sub;
};

#endif
