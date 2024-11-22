#include "tracker.hpp"

namespace atum {
Tracker::Tracker(const Logger::LoggerLevel loggerLevel) :
    logger{loggerLevel} {}

void Tracker::setPosition(const Position &iPosition) {
  positionMutex.take(10);
  position = iPosition;
  positionMutex.give();
}

Position Tracker::getPosition() {
  positionMutex.take(10);
  Position current = position;
  positionMutex.give();
  return current;
}
} // namespace atum