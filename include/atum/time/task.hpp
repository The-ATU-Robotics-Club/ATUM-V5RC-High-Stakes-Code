#pragma once

#include "../utility/logger.hpp"
#include "api.h"

namespace atum {
#define TASK_DEFINITIONS_FOR(handler)                                          \
  std::string handler::getHandlerName() const {                                \
    return #handler;                                                           \
  }                                                                            \
  void handler::prepBackgroundTasks()
#define START_TASK_1(name) taskParams.push_back({name, TASK_PRIORITY_DEFAULT, [=](){
#define START_TASK_2(name, priority) taskParams.push_back({name, priority, [=](){
#define GET_START_TASK_MACRO(_1, _2, NAME, ...) NAME
#define START_TASK(...)                                                        \
  GET_START_TASK_MACRO(__VA_ARGS__, START_TASK_2, START_TASK_1)(__VA_ARGS__)
#define END_TASK                                                               \
  }                                                                            \
  }                                                                            \
  );
#define TASK_BOILERPLATE()                                                     \
  friend class Task;                                                           \
  std::string getHandlerName() const override;                                 \
  void prepBackgroundTasks() override

class Task {
  public:
  struct TaskParams {
    const std::string name;
    const std::uint32_t priority;
    const std::function<void()> taskFn;
  };

  Task() = delete;
  Task(const Task &) = delete;
  Task(Task &&) = delete;

  template <typename TaskHandler>
  Task(TaskHandler *handler,
       const Logger::Level loggerLevel = Logger::Level::Info) :
      taskLogger{loggerLevel} {
    handler->prepBackgroundTasks();
    taskLogger.debug("Tasks associated with " + handler->getHandlerName() +
                     " have been prepared to start.");
  }

  void startBackgroundTasks();

  void stopBackgroundTasks();

  protected:
  virtual std::string getHandlerName() const = 0;
  virtual void prepBackgroundTasks() = 0;

  std::vector<TaskParams> taskParams;

  private:
  std::vector<std::unique_ptr<pros::Task>> tasks;
  Logger taskLogger; // Named differently than normal for simple disambiguation.
};
} // namespace atum