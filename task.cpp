#include "task.hpp"

namespace atum {
void Task::startBackgroundTasks() {
  task = std::make_unique<pros::Task>([this]() { backgroundTask(); },
                                      params.priority,
                                      TASK_STACK_DEPTH_DEFAULT,
                                      params.name.c_str());
}

void Task::stopBackgroundTasks() {
  task->remove();
  task = nullptr;
}
} // namespace atum