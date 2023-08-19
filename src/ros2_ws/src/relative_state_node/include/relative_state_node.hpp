#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class RelativeStateNode : public Component
{
    public:
        RelativeStateNode();
    private:
        Interface::string_sub_t relative_state_sub;
};
