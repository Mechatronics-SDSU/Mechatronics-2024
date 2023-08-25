#include "task_state.hpp"

TaskState::TaskState(const Task& task) : task{task} {}

TaskState::TaskState(const TaskState& nextState) : task{nextState.task} {}
