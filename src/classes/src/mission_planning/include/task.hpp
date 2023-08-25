#pragma once
#include "mission.hpp"
#include "task_state.hpp"

class Task;
class TaskState;
class Mission;

class Task
{
    public:
        Task(const Mission& mission);
        Task(const Task& nextTask);
        virtual ~Task() {};
        virtual void perform(const Task& Task) = 0;
        virtual void transition(const TaskState& nextState) = 0;
    protected:
        const Mission& mission;
        std::unique_ptr<TaskState> state;
};

