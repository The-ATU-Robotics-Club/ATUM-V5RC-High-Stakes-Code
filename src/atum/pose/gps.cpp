#include "gps.hpp"

namespace atum {
GPS::GPS(const std::int8_t port,
         const Pose &offset,
         const double iHeadingTrust,
         const double iFullPoseTrust,
         const Logger::Level loggerLevel) :
    headingOffset{offset.h},
    headingTrust{iHeadingTrust},
    fullPoseTrust{iFullPoseTrust},
    logger{loggerLevel} {
  gps = std::make_unique<pros::GPS>(port);
  check();
  initializeGPS(offset);
}

GPS::GPS(const Pose &offset,
         const double iHeadingTrust,
         const double iFullPoseTrust,
         const Logger::Level loggerLevel) :
    headingOffset{offset.h},
    headingTrust{iHeadingTrust},
    fullPoseTrust{iFullPoseTrust},
    logger{loggerLevel} {
  const auto gpsSensors{pros::GPS::get_all_devices()};
  if(!gpsSensors.size()) {
    logger.error("GPS not found!");
    gps = std::make_unique<pros::GPS>(errorPort);
    return;
  } else if(gpsSensors.size() > 1) {
    logger.warn("Multiple GPS sensors found! Using first port found.");
  }
  gps = std::make_unique<pros::GPS>(gpsSensors.front().get_port());
  initializeGPS(offset);
}

void GPS::setPose(const Pose &pose) {
  const double x{getValueAs<meter_t>(pose.x)};
  const double y{getValueAs<meter_t>(pose.y)};
  const double h{
      getValueAs<degree_t>(constrain180(pose.h + headingOffset) + 180_deg)};
  gps->set_position(x, y, h);
}

Pose GPS::getPose() {
  const meter_t x{gps->get_position_x()};
  const meter_t y{gps->get_position_y()};
  if(logger.getLevel() == Logger::Level::Debug) {
    GUI::Map::addPosition({x, y}, GUI::SeriesColor::Yellow);
  }
  return {x, y};
}

degree_t GPS::getHeading(const degree_t otherHeading) {
  if(!check()) {
    return otherHeading;
  }
  const degree_t reading{gps->get_yaw()};
  logger.debug("GPS heading reading is " + to_string(reading) + ".");
  const degree_t gpsHeading{reading - headingOffset};
  return headingTrust * gpsHeading + (1.0 - headingTrust) * otherHeading;
}

void GPS::resetTracker(Tracker *tracker) {
  if(!check() || gps->get_error() >= maxError) {
    return;
  }
  const Pose currentPose{getPose()};
  Pose newPose{fullPoseTrust * currentPose +
               (1.0 - fullPoseTrust) * tracker->getPose()};
  // Don't reset anything but x and y coordinates.
  newPose.h = currentPose.h;
  newPose.v = currentPose.v;
  newPose.a = currentPose.a;
  newPose.omega = currentPose.omega;
  newPose.alpha = currentPose.alpha;
  newPose.t = currentPose.t;
  tracker->setPose(newPose);
}

bool GPS::check() {
  const bool installed{gps->is_installed()};
  if(!installed) {
    logger.error("GPS on port " + std::to_string(gps->get_port()) +
                 " is not installed!");
  }
  return installed;
}

void GPS::initializeGPS(const UnwrappedPose &offset) {
  gps->set_data_rate(5);
  gps->set_offset(offset.x, offset.y);
}
} // namespace atum