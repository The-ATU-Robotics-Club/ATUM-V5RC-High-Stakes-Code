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
  public:                                                                      \
  friend class Task;                                                           \
                                                                               \
  protected:                                                                   \
  std::string getHandlerName() const;                                          \
  void prepBackgroundTasks()

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

  template <class TaskHandler>
  Task(TaskHandler *handler,
       const Logger::Level loggerLevel = Logger::Level::Info) :
      handlerName{handler->getHandlerName()}, taskLogger{loggerLevel} {
    handler->prepBackgroundTasks();
    taskLogger.debug("Tasks associated with " + handlerName +
                     " have been prepared to start.");
  }

  void startBackgroundTasks();

  void stopBackgroundTasks();

  protected:
  std::vector<TaskParams> taskParams;
  const std::string handlerName;

  private:
  std::vector<std::unique_ptr<pros::Task>> tasks;
  Logger taskLogger; // Named differently than normal for simple disambiguation.
};
} // namespace atum