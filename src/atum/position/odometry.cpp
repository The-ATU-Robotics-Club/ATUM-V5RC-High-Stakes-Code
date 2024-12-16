#include "odometry.hpp"

namespace atum {
Odometry::Odometry(std::unique_ptr<Odometer> iForward,
                   std::unique_ptr<Odometer> iSide,
                   std::unique_ptr<IMU> iImu,
                   Logger::Level loggerLevel) :
    Tracker(loggerLevel),
    Task(this, loggerLevel),
    forward{std::move(iForward)},
    side{std::move(iSide)},
    imu{std::move(iImu)} {
  if(!forward) logger.error("The forward odometer must be provided.");
  if(!side) logger.warn("The side odometer was not provided.");
  if(!imu) logger.error("An IMU must be provided.");
  logger.info("Odometry constructed!");
}

Position Odometry::update() {
  inch_t dxR{side->traveled()};
  inch_t dyR{forward->traveled()};
  const radian_t dh{imu->getTraveled()};
  const double dhScalar{getValueAs<radian_t>(dh)};
  scalar_t sinDHOverDH;
  scalar_t cosDHMinusOneOverDH;
  if(dh) {
    dxR = 2.0 * sin(dh / 2.0) * (dxR / dhScalar + side->getFromCenter());
    dyR = 2.0 * sin(dh / 2.0) * (dyR / dhScalar + forward->getFromCenter());
    sinDHOverDH = sin(dh) / dhScalar;
    cosDHMinusOneOverDH = (cos(dh) - 1.0) / dhScalar;
  } else {
    sinDHOverDH = 1.0 - (dhScalar * dhScalar) / 6.0;
    cosDHMinusOneOverDH = -dhScalar / 2.0;
  }
  const inch_t dxG{sinDHOverDH * dxR + cosDHMinusOneOverDH * dyR};
  const inch_t dyG{-cosDHMinusOneOverDH * dxR + sinDHOverDH * dyR};
  /*
  if(headingChange) {
    sideChange = 2.0 * sin(headingChange / 2.0) *
                 (sideChange / getValueAs<radian_t>(headingChange) +
                  side->getFromCenter());
    forwardChange = 2.0 * sin(headingChange / 2.0) *
                    (forwardChange / getValueAs<radian_t>(headingChange) +
                     forward->getFromCenter());
  }*/
  return integratePosition(dxG, dyG, dh);
}

Position Odometry::integratePosition(inch_t dx, inch_t dy, radian_t dh) {
  if(!std::isfinite(getValueAs<inch_t>(dx)) ||
     !std::isfinite(getValueAs<inch_t>(dy)) ||
     !std::isfinite(getValueAs<radian_t>(dh))) {
    logger.error("Invalid values read from odometers.");
    dx = 0_in;
    dy = 0_in;
    dh = 0_rad;
  }
  Position currentPosition{getPosition()};
  const radian_t averageHeading{currentPosition.h + 0.5 * dh};
  currentPosition.x += cos(averageHeading) * dx + sin(averageHeading) * dy;
  currentPosition.y += -sin(averageHeading) * dx + cos(averageHeading) * dy;
  currentPosition.h += dh;
  setPosition(currentPosition);
  return getPosition(); // Use getPosition() for logging purposes.
}

TASK_DEFINITIONS_FOR(Odometry) {
  START_TASK("Odometry Loop", TASK_PRIORITY_MAX)
  while(true) {
    update();
    wait(10_ms);
  }
  END_TASK
}
} // namespace atum