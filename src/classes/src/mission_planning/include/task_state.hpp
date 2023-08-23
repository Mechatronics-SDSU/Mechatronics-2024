#pragma once
#include "task.hpp"

class Task;

class TaskState
{
    public:
        TaskState(const Task& task);
        TaskState(const TaskState& taskState);
        virtual ~TaskState() {};
        virtual void checkConditions() = 0;
        virtual void perform(const Task& task) = 0;
    protected:
        const Task& task;
};
