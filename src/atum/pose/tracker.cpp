#include "tracker.hpp"

namespace atum {
Tracker::Tracker(const Logger::Level loggerLevel) : logger{loggerLevel} {}

void Tracker::setPose(const Pose &iPose) {
  std::scoped_lock lock{poseMutex};
  pose = iPose;
}

Pose Tracker::getPose() {
  std::scoped_lock lock{poseMutex};
  Pose current = pose;
  logger.debug("Current tracker pose is " + toString(pose) + ".");
  return current;
}
} // namespace atum