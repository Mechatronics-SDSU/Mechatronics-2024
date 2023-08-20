#ifndef RELATIVE_STATE_NODE_H
#define RELATIVE_STATE_NODE_H

#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class RelativeStateNode : public Component
{
    public:
        RelativeStateNode();
        void publishRelativeState();
        void resetRelativeState (const Interface::trigger_request_t request, const Interface::trigger_response_t response);
        void resetRelativePosition (const Interface::trigger_request_t request, const Interface::trigger_response_t response);
        void absoluteStateSubCallback(const scion_types::msg::State::SharedPtr msg);
    private:
        Interface::RobotState               relative_robot_state;
        Interface::ros_trigger_service_t    reset_relative_state_service;
        Interface::ros_trigger_service_t    reset_relative_position_service;
        Interface::state_pub_t              relative_state_pub;
        Interface::state_sub_t              absolute_state_sub;
};

#endif
