#ifndef SUB_STATE_LISTENER_NODE_H
#define SUB_STATE_LISTENER_NODE_H

#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class SubStateListenerNode : public Component
{
    public:
        SubStateListenerNode();
    private:
      Interface::sub_state_sub_t submarine_state_sub;
};

#endif