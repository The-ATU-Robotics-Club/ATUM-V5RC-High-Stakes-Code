#pragma once

#include "../time/timer.hpp"
#include "../utility/logger.hpp"
#include "position.hpp"

namespace atum {
class Tracker {
  public:
  Tracker(const Logger::Level loggerLevel = Logger::Level::Info);

  virtual Position update() = 0;

  virtual void setPosition(const Position &iPosition);

  virtual Position getPosition();

  protected:
  Logger logger;
  Position position{0_tile, 0_tile, 0_deg};
  pros::Mutex positionMutex;
};
} // namespace atum