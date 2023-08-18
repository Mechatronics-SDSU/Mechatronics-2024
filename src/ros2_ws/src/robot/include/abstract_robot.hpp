#include "robot_interface.hpp"
#include "can_interface.hpp"

class AbstractRobot
{
    public: 
        virtual void main_update_loop() = 0;
        virtual std::string getName() = 0;
        virtual int getMotorCount() = 0;
        virtual Interface::matrix_t getThrustMapper() = 0;
        virtual std::shared_ptr<CanInterface::CanClient> getCanClient() = 0;
};