#ifndef ROBOT_H
#define ROBOT_H

#include <memory>
#include "robot_interface.hpp"

class Robot;
class Percy;
class Junebug;
class Configuration;

class Robot
{
    public: 
        Robot(const Configuration& configuration);
        void run_update_loop();
    private:
        char* name_;
        int motor_count_;
};

class Percy : Robot
{
    public:
        Percy();
};

class Junebug : Robot
{
    public:
        Junebug();
};


#endif // ROBOT_H