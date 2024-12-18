#pragma once

#include "../time/timer.hpp"
#include "../utility/logger.hpp"
#include "pose.hpp"

namespace atum {
/**
 * @brief This class creates an interface for any system that can
 * do position tracking.
 *
 */
class Tracker {
  public:
  /**
   * @brief Constructs a new tracker and performs logging.
   *
   * @param loggerLevel
   */
  Tracker(const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief This method should update the current tracked position.
   * It should be overriden in derivatives of Tracker.
   *
   * @return Pose
   */
  virtual Pose update() = 0;

  /**
   * @brief Sets the current position of the tracker.
   *
   * @param iPosition
   */
  virtual void setPosition(const Pose &iPosition);

  /**
   * @brief Gets the current position of the tracker.
   *
   * @return Pose
   */
  virtual Pose getPosition();

  protected:
  Logger logger;
  Pose position{0_tile, 0_tile, 0_deg};
  pros::Mutex positionMutex;
};
} // namespace atum