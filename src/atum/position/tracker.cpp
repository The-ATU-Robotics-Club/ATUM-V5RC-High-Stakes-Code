#include "tracker.hpp"

namespace atum {
Tracker::Tracker(const Logger::Level loggerLevel) : logger{loggerLevel} {}

void Tracker::setPosition(const Pose &iPosition) {
  std::scoped_lock lock{positionMutex};
  position = iPosition;
}

Pose Tracker::getPosition() {
  std::scoped_lock lock{positionMutex};
  Pose current = position;
  logger.debug("Current tracker position is " + toString(position) + ".");
  return current;
}
} // namespace atum