#ifndef ROBOTS_H
#define ROBOTS_H

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
        const char* name_ = "percy";
        const int motor_count_ = 8;
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

#endif // ROBOTS_H