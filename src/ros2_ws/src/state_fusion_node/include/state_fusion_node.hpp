#include "robot_interface.hpp"
#include "component.hpp"
#include <memory>

class StateFusionNode : public Component
{
    public:
        StateFusionNode();
    private:
        Interface::string_sub_t state_fusion_sub;
};
