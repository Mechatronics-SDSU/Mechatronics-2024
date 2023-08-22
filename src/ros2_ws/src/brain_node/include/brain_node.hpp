#ifndef BRAIN_NODE_H
#define BRAIN_NODE_H

#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class BrainNode : public Component
{
    public:
        BrainNode();
    private:
        Interface::string_sub_t brain_sub;
};

#endif
