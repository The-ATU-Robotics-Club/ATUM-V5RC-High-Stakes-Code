#include "tracker.hpp"


namespace atum {
Tracker::Tracker(const Logger::Level loggerLevel,
                 const GUI::SeriesColor iDotColor) :
    logger{loggerLevel},
    dotColor{iDotColor} {}

void Tracker::setPose(const Pose &iPose) {
  pose = iPose;
}

Pose Tracker::getPose() {
  if(logger.getLevel() >= Logger::Level::Info) {
    GUI::Map::addPosition(pose, dotColor);
  }
  logger.debug("Tracker pose: " + toString(pose) + ".");
  return pose;
}
} // namespace atum