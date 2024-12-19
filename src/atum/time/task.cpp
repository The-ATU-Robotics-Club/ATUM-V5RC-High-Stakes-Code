#include "task.hpp"

namespace atum {
void Task::startBackgroundTasks() {
  // If the tasks have already started, don't try to start them again.
  if(tasks.size()) {
    return;
  }
  for(TaskParams params : taskParams) {
    auto task = std::make_unique<pros::Task>(params.taskFn,
                                             params.priority,
                                             TASK_STACK_DEPTH_DEFAULT,
                                             params.name.c_str());
    tasks.push_back(std::move(task));
    taskLogger.info("Task \"" + params.name + "\" with priority " +
                    std::to_string(params.priority) + " has started.");
  }
}

void Task::stopBackgroundTasks() {
  tasks.clear();
  taskLogger.info("Tasks associated with \"" + handlerName +
                  "\" have been stopped.");
}
} // namespace atum