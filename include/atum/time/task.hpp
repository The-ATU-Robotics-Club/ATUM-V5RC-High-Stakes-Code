#pragma once

#include "api.h"

namespace atum {
class Task {
  public:
  struct TaskParams {
    std::string name{""};
    std::uint32_t priority{TASK_PRIORITY_DEFAULT};
    std::uint16_t depth{TASK_STACK_DEPTH_DEFAULT};
  };

  Task(const TaskParams &iParams);

  void startBackgroundTask();

  void stopBackgroundTask();

  private:
  virtual void backgroundTask() = 0;

  std::unique_ptr<pros::Task> task;
  const TaskParams params;
};
} // namespace atum