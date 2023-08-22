#ifndef TRANSLATOR_NODE_H
#define TRANSLATOR_NODE_H

#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class TranslatorNode : public Component
{
    public:
        TranslatorNode();
    private:
        Interface::string_sub_t translator_sub;
};

#endif
