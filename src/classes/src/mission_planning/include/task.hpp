#pragma once

class Task;
class TaskState;

class TaskState
{
    public:
        virtual ~TaskState() {}
        virtual void perform(Task& task) = 0;
};

class Task
{
    public:
        Task(std::string name);
        virtual ~Task() {};
        virtual void perform(Task& Task);
    protected:
        std::unique_ptr<TaskState> state;
};