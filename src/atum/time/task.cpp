#include "task.hpp"

namespace atum {
Task::Task(const TaskParams &iParams) : params{iParams} {}

void Task::startBackgroundTask() {
  task = std::make_unique<pros::Task>([this]() { backgroundTask(); },
                                      params.priority,
                                      params.depth,
                                      params.name.c_str());
}

void Task::stopBackgroundTask() {
  task->remove();
  task = nullptr;
}
} // namespace atum