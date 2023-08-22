#pragma once
#include <queue>
#include "task.hpp"

class Mission
{
    public:
        Mission(std::string& mission_plan);
    protected:
        Task* currentTask;
        std::queue<Task> tasks;
};