/**
 * @file schedule.hpp
 * @brief Includes the Scheduler class.
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "task.hpp"
#include "timer.hpp"
#include <queue>

namespace atum {
/**
 * @brief This class serves as a way to perform actions once certain conditions
 * are met. Items will be interrupted if the competition state changes or the
 * object goes out of scope. Actions should be fairly simple.
 *
 */
class Scheduler : public Task {
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
    /**
     * @brief Constructs a new scheduler item. 
     * 
     * @param iName 
     * @param iCondition 
     * @param iTodo 
     * @param iTimeout 
     * @param iTodoTimeout 
     */
    Item(const std::string &iName,
         const Condition &iCondition,
         const std::function<void()> &iTodo,
         const second_t &iTimeout = forever,
         const std::optional<std::function<void()>> iTodoTimeout = {});

    std::string name;
    Condition condition;
    std::function<void()> todo;
    // The time before the scheduler gives up and runs timeout action.
    Timer timeout;
    // Unless provided, the default timeout action is the todo action.
    std::optional<std::function<void()>> todoTimeout{};
  };

  /**
   * @brief Constructs a new Scheduler object.
   *
   * @param loggerLevel
   */
  Scheduler(const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Cleans up the scheduled action and stops the background task
   * associated with it.
   *
   */
  ~Scheduler();

  /**
   * @brief Schedules an item to be performed when its criteria are met.
   *
   * @param toSchedule
   */
  void schedule(const Scheduler::Item &toSchedule);

  private:
  /**
   * @brief Higher than standard delay to allow several scheduled items at once
   * with little impact.
   *
   */
  static constexpr second_t schedulerLoopDelay{20_ms};

  std::queue<Item> scheduled;
  Logger logger;
};
} // namespace atum