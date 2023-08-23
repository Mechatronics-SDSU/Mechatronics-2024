#pragma once
#include <queue>
#include <memory>
#include "task.hpp"

class Task;

class Mission
{
    public:
        Mission(std::string& mission_plan);
        void addTask(std::unique_ptr<Task> task);
        void execute(Task& task);
        void transition();
    protected:
        std::unique_ptr<Task> currentTask;
        std::queue<std::unique_ptr<Task>> tasks;
};