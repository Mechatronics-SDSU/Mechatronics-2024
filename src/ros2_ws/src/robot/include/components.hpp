#include "robot.hpp"
#include "component.hpp"

class Components
{
    public:
        Components(Robot& robot);
        std::shared_ptr<std::vector<std::shared_ptr<Component>>> CreateComponentVector();
    private:
        const Robot& robot;
};