#include "task.hpp"

Task::Task(const Mission& mission) : mission{mission} {}

Task::Task(const Task& nextTask) : mission{nextTask.mission} {}
