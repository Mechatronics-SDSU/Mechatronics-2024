#ifndef ROBOT_H
#define ROBOT_H

#include <memory>

class Robot;
class Percy;
class Junebug;
class Config;

class Robot
{
    public: 
        Robot();
    private:
        const std::unique_ptr<Config> config;
        const char* name_;
        const int motor_count_;
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

class Config
{
    public:
        Config();
};

#endif // ROBOT_H