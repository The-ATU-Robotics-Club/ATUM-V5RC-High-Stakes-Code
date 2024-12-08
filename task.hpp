#pragma once

#include "api.h"

namespace atum {
#define TASK_DEFINITIONS_FOR(handler) void handler::prepBackgroundTasks()
#define START_TASK_1(name) taskParams.push_back({name, TASK_PRIORITY_DEFAULT, [=](){
#define START_TASK_2(name, priority) taskParams.push_back({name, priority, [=](){
#define GET_START_TASK_MACRO(_1, _2, NAME, ...) NAME
#define START_TASK(...) GET_MACRO(__VA_ARGS__, START_TASK_2, START_TASK_1)(__VA_ARGS__)
#define END_TASK                                                               \
  }                                                                            \
  })

class Task {
  public:
  struct TaskParams {
    const std::string name;
    const std::uint32_t priority;
    const std::function<void()> taskFn;
  };

  Task() = delete;

  template <typename TaskHandler>
  Task(TaskHandler *handler) {
    handler->prepBackgroundTasks();
  }

  void startBackgroundTasks();

  void stopBackgroundTasks();

  private:
  virtual void prepBackgroundTasks() = 0;

  std::vector<TaskParams> taskParams;
  std::vector<std::unique_ptr<pros::Task>> tasks;
};
} // namespace atum