/**
 * @file turn.hpp
 * @brief Includes the Turn class.
 * @date 2025-01-10
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../controllers/pid.hpp"
#include "../systems/drive.hpp"
#include "../utility/acceptable.hpp"
#include "movement.hpp"

namespace atum {
/**
 * @brief Encapsulates the logic behind turns.
 *
 */
class Turn : public Movement {
  public:
  /**
   * @brief Constructs a new Turn object.
   *
   * @param iDrive
   * @param iPID
   * @param iAcceptable
   * @param loggerLevel
   */
  Turn(Drive *iDrive,
       const PID &iPID,
       const AcceptableAngle &iAcceptable,
       const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Points towards the given target angle.
   *
   * @param timeout
   * @param target
   * @param maxVoltage
   */
  void toward(const second_t timeout,
              const Pose &target,
              const double maxVoltage = Motor::maxVoltage);

  /**
   * @brief Points towards the given target pose.
   *
   * @param timeout
   * @param target
   * @param maxVoltage
   */
  void toward(const second_t timeout,
              const degree_t target,
              const double maxVoltage = Motor::maxVoltage);

  /**
   * @brief Points away from the given target pose.
   *
   * @param timeout
   * @param target
   * @param maxVoltage
   */
  void awayFrom(const second_t timeout,
                const Pose &target,
                const double maxVoltage = Motor::maxVoltage);

  /**
   * @brief Points away from the given target angle.
   *
   * @param timeout
   * @param target
   * @param maxVoltage
   */
  void awayFrom(const second_t timeout,
                const degree_t target,
                const double maxVoltage = Motor::maxVoltage);

  private:
  Drive *drive;
  PID pid;
  AcceptableAngle acceptable;
  Logger logger;
};
} // namespace atum