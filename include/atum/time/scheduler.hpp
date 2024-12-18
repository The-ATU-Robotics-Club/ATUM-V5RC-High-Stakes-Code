#pragma once

#include "task.hpp"
#include "time.hpp"
#include <queue>

namespace atum {
class Scheduler : public Task {
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

  Scheduler(const Logger::Level loggerLevel = Logger::Level::Info);

  void schedule(const Item &item);

  void deschedule();

  private:
  std::queue<Item> items;
};
} // namespace atum