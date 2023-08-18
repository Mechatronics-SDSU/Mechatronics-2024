#pragma once
#include "robot_interface.hpp"
#include "component.hpp"

class GUI_Listener : public Component
{
    public:
        GUI_Listener();
    private:
        Interface::string_sub_t gui_subscriber;
        void gui_subscription_callback();
};
