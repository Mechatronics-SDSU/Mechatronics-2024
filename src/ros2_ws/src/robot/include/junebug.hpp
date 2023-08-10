#pragma once
#include "robot.hpp"

class Junebug : public Robot
{
    public:
        Junebug(const Configuration& configuration);
        static std::shared_ptr<Robot> CreateJunebug(const Configuration& configuration);
};