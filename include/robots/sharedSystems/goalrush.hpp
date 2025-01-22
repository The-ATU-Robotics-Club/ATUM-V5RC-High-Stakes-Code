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
class GoalRush {
  public:
  GoalRush(std::unique_ptr<Piston> iArm,
           std::unique_ptr<Piston> iClamp,
           const Logger::Level loggerLevel = Logger::Level::Info);

  void extendArm();

  void retractArm();

  void grab();

  void release();

  void toggleArm();

  void toggleClamp();

  private:
  std::unique_ptr<Piston> arm;
  std::unique_ptr<Piston> clamp;
  Logger logger;
};
} // namespace atum
