/**
 * @file moveTo.hpp
 * @brief Includes the MoveTo class.
 * @date 2025-02-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "turn.hpp"

namespace atum {
/**
 * @brief Encapsulates the logic behind point-to-point movements.
 *
 */
class MoveTo : public Movement {
  public:
  /**
   * @brief Constructs a new MoveTo object.
   *
   * Turn is used for the initial turn toward the target. The direction
   * controller is used to steer the drive while moving to the target
   * position. Turn to threshold refers to how close the drive has to be to the
   * target to no longer turn toward it.
   *
   * @param iDrive
   * @param iTurn
   * @param iLateralPID
   * @param iDirectionPID
   * @param iTurnToThreshold
   * @param iAcceptable
   * @param loggerLevel
   */
  MoveTo(Drive *iDrive,
         Turn *iTurn,
         const PID &iLateralPID,
         const PID &iDirectionPID,
         const AcceptableDistance &iAcceptable,
         const meter_t iTurnToThreshold = 0.5_tile,
         const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Moves towards the given target position, going forward.
   *
   * @param timeout
   * @param target
   * @param maxVoltage
   */
  void forward(const second_t timeout,
               Pose target,
               const double maxVoltage = Motor::maxVoltage);

  /**
   * @brief Moves towards the given target position, going in reverse.
   *
   * @param timeout
   * @param target
   * @param maxVoltage
   */
  void reverse(const second_t timeout,
               Pose target,
               const double maxVoltage = Motor::maxVoltage);

  private:
  /**
   * @brief Contains the basic behavior for moving to a position on the field,
   * accounting for if we are reversing or not.
   *
   * @param target
   * @param timeout
   * @param maxVoltage
   * @param reversed
   */
  void moveToPoint(second_t timeout,
                   Pose target,
                   const double maxVoltage,
                   const bool reversed);

  Drive *drive;
  Turn *turn;
  PID lateralPID;
  PID directionPID;
  AcceptableDistance acceptable;
  const meter_t turnToThreshold;
  Logger logger;
  second_t startTime;
};
} // namespace atum