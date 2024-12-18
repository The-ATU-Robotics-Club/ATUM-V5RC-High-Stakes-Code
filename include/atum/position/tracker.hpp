#pragma once

#include "../time/timer.hpp"
#include "../utility/logger.hpp"
#include "pose.hpp"

namespace atum {
class Tracker {
  public:
  Tracker(const Logger::Level loggerLevel = Logger::Level::Info);

  virtual Pose update() = 0;

  virtual void setPosition(const Pose &iPosition);

  virtual Pose getPosition();

  protected:
  Logger logger;
  Pose position{0_tile, 0_tile, 0_deg};
  pros::Mutex positionMutex;
};
} // namespace atum