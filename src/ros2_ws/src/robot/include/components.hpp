#include "robot.hpp"
#include "component.hpp"

namespace Components
{
    std::shared_ptr<std::vector<std::shared_ptr<Component>>> CreateComponentVector(Robot& robot);
};