/**
 * @file goalRush.hpp
 * @brief Includes the GoalRush class.
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "atum/atum.hpp"

namespace atum {
/**
 * @brief This class encapsulates the logic behind the goal rush mechanism.
 *
 */
class GoalRush {
  public:
  /**
   * @brief Constructs a new GoalRush object.
   *
   * @param iArm
   * @param iLimitSwitch
   * @param loggerLevel
   */
  GoalRush(std::unique_ptr<Piston> iArm,
           std::unique_ptr<LimitSwitch> iLimitSwitch,
           const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Extends the arm of the goal rush.
   *
   */
  void extend();

  /**
   * @brief Retracts the arm of the goal rush.
   *
   */
  void retract();

  /**
   * @brief Toggles whether the arm is down or up.
   *
   */
  void toggle();

  /**
   * @brief Returns whether the arm is up or not.
   *
   * @return true
   * @return false
   */
  bool isUp() const;

  /**
   * @brief Returns whether a goal is detected in the rush or not.
   *
   * @return true
   * @return false
   */
  bool hasGoal() const;

  private:
  std::unique_ptr<Piston> arm;
  std::unique_ptr<LimitSwitch> limitSwitch;
  Logger logger;
};
} // namespace atum
