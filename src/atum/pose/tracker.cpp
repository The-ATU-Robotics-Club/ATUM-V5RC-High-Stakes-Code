#include "tracker.hpp"

namespace atum {
Tracker::Tracker(const Logger::Level loggerLevel) : logger{loggerLevel} {}

void Tracker::setPose(const Pose &iPose) {
  pose = iPose;
}

Pose Tracker::getPose() {
  Pose current = pose;
  logger.debug("Current tracker pose is " + toString(pose) + ".");
  return current;
}
} // namespace atum