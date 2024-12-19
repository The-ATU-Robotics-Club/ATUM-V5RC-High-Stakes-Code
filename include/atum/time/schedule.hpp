#pragma once

#include "task.hpp"
#include "timer.hpp"
#include <queue>

namespace atum {
class Schedule : public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  struct Item {
    const std::string name;
    const Condition condition;
    const std::function<void()> todo;
    // The time before the scheduler gives up and runs timeout action. Zero
    // seconds indicates it will never gives up.
    const second_t timeout{0_s};
    // Default timeout action to be nothing.
    const std::function<void()> todoTimeout{[]() {}};
  };

  Schedule(const Item &iItem,
           const Logger::Level loggerLevel = Logger::Level::Info);

  private:
  Item item;
  Logger logger;
};
} // namespace atum