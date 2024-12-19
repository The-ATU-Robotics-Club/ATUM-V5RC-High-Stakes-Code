#pragma once

#include "../../pros/misc.hpp"
#include "task.hpp"
#include "timer.hpp"
#include <queue>

namespace atum {
/**
 * @brief This class serves as a way to perform actions once certain conditions
 * are met. Construct an anonymous object with the necessary parameters for a
 * schedule item (e.g., Schedule{{...}}; ). Will be interrupted if competition
 * state changes. Actions should be fairly simple.
 *
 */
class Schedule : public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  /**
   * @brief Useful value to have whenever you are only interested in the timeout
   * action. Does what it says on the tin when called.
   *
   */
  static const std::function<void()> doNothing;

  /**
   * @brief Useful value to have whenever you are only interested in the timeout
   * action. Does what it says on the tin when called.
   *
   */
  static const Condition neverMet;

  /**
   * @brief The necessary parameters for a schedule item. Once the condition is
   * true, the todo method will be ran.
   *
   */
  struct Item {
    const std::string name;
    const Condition condition;
    const std::function<void()> todo;
    // The time before the scheduler gives up and runs timeout action. Zero
    // seconds indicates it will never gives up.
    const second_t timeout{0_s};
    // Default timeout action to be nothing.
    const std::function<void()> todoTimeout{doNothing};
  };

  /**
   * @brief Schedules an item with an action to perform once a condition is met.
   *
   * @param iItem
   * @param loggerLevel
   */
  Schedule(const Item &iItem,
           const Logger::Level loggerLevel = Logger::Level::Info);

  private:
  Item item;
  Logger logger;
};
} // namespace atum