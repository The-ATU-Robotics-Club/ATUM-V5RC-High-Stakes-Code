#include "tracker.hpp"

namespace atum {
Tracker::Tracker(const Logger::Level loggerLevel) : logger{loggerLevel} {}

void Tracker::setPose(const Pose &iPose) {
  pose = iPose;
}

Pose Tracker::getPose() {
  if(logger.getLevel() == Logger::Level::Debug) {
    GUI::Map::addPosition(pose, GUI::SeriesColor::Green);
  }
  return pose;
}
} // namespace atum