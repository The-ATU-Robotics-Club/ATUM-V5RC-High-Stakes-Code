/**
 * @file moveTo.hpp
 * @brief Includes the MoveTo class.
 * @date 2025-02-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../controllers/pid.hpp"
#include "../systems/drive.hpp"
#include "profileFollower.hpp"
#include "turn.hpp"

namespace atum {
/**
 * @brief Encapsulates the logic behind point-to-point movements. Mostly
 * combines the drive with the profile follower class.
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
   * @param iFollower
   * @param iDirectionController
   * @param iTurnToThreshold
   * @param loggerLevel
   */
  MoveTo(Drive *iDrive,
         Turn *iTurn,
         std::unique_ptr<LateralProfileFollower> iFollower,
         std::unique_ptr<PID> iDirectionController,
         const meter_t iTurnToThreshold = 1_tile,
         const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Moves towards the given target position, going forward.
   *
   * @param target
   * @param specialParams
   */
  void forward(Pose target,
               const LateralProfile::Parameters &specialParams = {});

  /**
   * @brief Moves towards the given target position, going in reverse.
   *
   * @param target
   * @param specialParams
   */
  void reverse(Pose target,
               const LateralProfile::Parameters &specialParams = {});

  private:
  Drive *drive;
  Turn *turn;
  std::unique_ptr<LateralProfileFollower> follower;
  std::unique_ptr<PID> directionController;
  const meter_t turnToThreshold;
  Logger logger;
};
} // namespace atum