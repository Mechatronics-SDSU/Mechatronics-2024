#ifndef QUEUE_NODE_H
#define QUEUE_NODE_H

#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class QueueNode : public Component
{
    public:
        QueueNode();
    private:
        Interface::string_sub_t queue_sub;
};

#endif
