/**
 * @file turn.hpp
 * @brief Includes the Turn class.
 * @date 2025-01-10
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../systems/drive.hpp"
#include "profileFollower.hpp"

namespace atum {
/**
 * @brief Encapsulates the logic behind turns. Mostly combines the drive with
 * the profile follower class.
 *
 */
class Turn {
  public:
  /**
   * @brief Constructs a new Turn object.
   *
   * @param iDrive
   * @param iFollower
   * @param loggerLevel
   */
  Turn(Drive *iDrive,
       std::unique_ptr<AngularProfileFollower> iFollower,
       const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Points towards the given target angle.
   *
   * @param target
   * @param specialParams
   */
  void toward(const Pose &target,
              const AngularProfile::Parameters &specialParams = {});

  /**
   * @brief Points towards the given target pose.
   *
   * @param target
   * @param specialParams
   */
  void toward(const degree_t target,
              const AngularProfile::Parameters &specialParams = {});

  /**
   * @brief Points away from the given target pose.
   *
   * @param target
   * @param specialParams
   */
  void awayFrom(const Pose &target,
                const AngularProfile::Parameters &specialParams = {});

  /**
   * @brief Points away from the given target angle.
   *
   * @param target
   * @param specialParams
   */
  void awayFrom(const degree_t target,
                const AngularProfile::Parameters &specialParams = {});

  /**
   * @brief Interrupts the current turn.
   *
   */
  void interrupt();

  /**
   * @brief Sets whether the turns should be flipped across the x-axis (if
   * the color is changed).
   *
   * @param iFlipped
   */
  void setFlipped(const bool iFlipped);

  private:
  Drive *drive;
  std::unique_ptr<AngularProfileFollower> follower;
  Logger logger;
  bool interrupted{false};
  bool flipped{false};
};
} // namespace atum