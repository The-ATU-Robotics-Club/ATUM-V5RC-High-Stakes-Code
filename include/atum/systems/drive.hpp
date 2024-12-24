/**
 * @file drive.hpp
 * @brief Includes the Drive class. 
 * @date 2024-12-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "../devices/motor.hpp"
#include "../pose/tracker.hpp"
#include "../utility/logger.hpp"
#include "../utility/misc.hpp"
#include "api.h"

namespace atum {
/**
 * @brief This class encapsulates all of the logic behind the drive.
 * Including tank and arcade controls, setting brake mode, and tracking
 * pose.
 *
 */
class Drive {
  public:
  /**
   * @brief Constructs a new drive. Requires the left, right, and tracker
   * to be provided.
   *
   * @param iLeft
   * @param iRight
   * @param iTracker
   * @param loggerLevel
   */
  Drive(std::unique_ptr<Motor> iLeft,
        std::unique_ptr<Motor> iRight,
        std::unique_ptr<Tracker> iTracker,
        const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Provides tank controls: the left voltage and right voltage are
   * applied directly to the corresponding sides of the drive.
   *
   * @param leftVoltage
   * @param rightVoltage
   */
  void tank(const double leftVoltage, const double rightVoltage);

  /**
   * @brief Provides arcade controls: the forward voltage makes the drive go
   * forward while the turn voltage causes it to turn.
   *
   * @param forwardVoltage
   * @param turnVoltage
   */
  void arcade(const double forwardVoltage, const double turnVoltage);

  /**
   * @brief Sets the current pose of the drive.
   *
   * @param iPose
   */
  void setPose(const Pose &iPose);

  /**
   * @brief Gets the current pose of the drive.
   *
   * @return Pose
   */
  Pose getPose() const;

  /**
   * @brief Sets the brake mode of the motors on the drive.
   *
   * @param brakeMode
   */
  void setBrakeMode(const pros::v5::MotorBrake brakeMode);

  /**
   * @brief Gets the brake mode of the motors on the drive.
   *
   * @return pros::v5::MotorBrake
   */
  pros::v5::MotorBrake getBrakeMode() const;

  private:
  std::unique_ptr<Motor> left;
  std::unique_ptr<Motor> right;
  std::unique_ptr<Tracker> tracker;
  Logger logger;
};
} // namespace atum