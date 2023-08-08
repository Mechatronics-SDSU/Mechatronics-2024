#pragma once
#include "robot.hpp"

class Percy : public Robot
{
    public:
        Percy(const Configuration& configuration);
        static std::unique_ptr<Robot> CreatePercy(const Configuration& configuration);
};