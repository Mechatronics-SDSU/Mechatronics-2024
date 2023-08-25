#include "mission.hpp"


Mission::Mission(std::string& mission_plan)
{
    this->transition();
}

void Mission::transition()
{
    std::unique_ptr<Task>& currentTask = tasks.front(); tasks.pop();
}

void Mission::addTask(std::unique_ptr<Task> task) 
{
    tasks.push(std::move(task));
}

void Mission::execute(Task& task) 
{
    task.perform(*currentTask);
}
