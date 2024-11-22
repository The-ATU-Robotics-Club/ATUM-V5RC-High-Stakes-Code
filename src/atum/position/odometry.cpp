#include "odometry.hpp"

namespace atum {
Odometry::Odometry(std::unique_ptr<Odometer> iForward,
                   std::unique_ptr<Odometer> iSide,
                   std::unique_ptr<IMU> iImu,
                   Logger::LoggerLevel loggerLevel) :
    Tracker(loggerLevel),
    Task({"Odometry", TASK_PRIORITY_MAX}),
    forward{std::move(iForward)},
    side{std::move(iSide)},
    imu{std::move(iImu)} {
  if(!forward) logger.error("The right odometer must be provided.");
  if(!imu) logger.error("An IMU must be provided.");
  logger.info("Odometry constructed!");
}

Position Odometry::update() {
  const auto [dl, dr, dh] = getDeltas();
  const auto [dxR, dyR] = getRelativeDeltas(dl, dr, dh);
  return integratePosition(dxR, dyR, dh);
}

std::tuple<inch_t, inch_t, radian_t> Odometry::getDeltas() {
  const inch_t dr{forward->traveled()};
  radian_t dh;
  dh = imu->getTraveled();
  return std::make_tuple(dr, dr, dh);
}

std::pair<inch_t, inch_t> Odometry::getRelativeDeltas(const inch_t dl,
                                                      const inch_t dr,
                                                      const radian_t dh) {
  inch_t dx{0.0_in};
  inch_t dy{0.0_in};
  const inch_t dd = (dl + dr) / 2.0;
  dx = sin(dh) * dd;
  dy = cos(dh) * dd;
  return std::make_pair(dx, dy);
}

Position Odometry::integratePosition(inch_t dx, inch_t dy, radian_t dh) {
  if(!std::isfinite(getValueAs<inch_t>(dx))) dx = 0_in;
  if(!std::isfinite(getValueAs<inch_t>(dy))) dy = 0_in;
  if(!std::isfinite(getValueAs<radian_t>(dh))) dh = 0_rad;
  Position currentPosition{getPosition()};
  Position previousPosition{currentPosition};
  const radian_t hAvg{currentPosition.h + 0.5 * dh};
  currentPosition.x += cos(hAvg) * dx + sin(hAvg) * dy;
  currentPosition.y += -sin(hAvg) * dx + cos(hAvg) * dy;
  currentPosition.h += dh;
  auto [v, w] = getVW(distance(previousPosition, currentPosition), dh);
  currentPosition.v = v;
  currentPosition.w = w;
  setPosition(currentPosition);
  logger.debug("Odometry Reading: " + toString(currentPosition));
  return currentPosition;
}

void Odometry::backgroundTask() {
  while(true) {
    update();
    wait(10_ms);
  }
}
} // namespace atum