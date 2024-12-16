#include "tracker.hpp"

namespace atum {
Tracker::Tracker(const Logger::Level loggerLevel) : logger{loggerLevel} {}

void Tracker::setPosition(const Position &iPosition) {
  std::scoped_lock lock{positionMutex};
  position = iPosition;
}

Position Tracker::getPosition() {
  std::scoped_lock lock{positionMutex};
  Position current = position;
  logger.debug("Current tracker position is " + to_string(position) + ".");
  return current;
}
} // namespace atum