#pragma once
#include "task.hpp"

class LookingForGate : public TaskState
{
    void checkConditions();
    void perform(const Task& task);
};

class CenteringOnGate : public TaskState
{
    void checkConditions();
    void perform(const Task& task);
};

class GoThroughGate : public Task
{
    void perform(const Task& Task);
    void transition(const TaskState& nextState);
};