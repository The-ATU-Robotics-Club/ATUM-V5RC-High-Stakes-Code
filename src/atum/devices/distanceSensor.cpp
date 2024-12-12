#include "distanceSensor.hpp"

namespace atum {
DistanceSensor::DistanceSensor(const std::int8_t port,
                               const millimeter_t iThreshold,
                               const Logger::Level loggerLevel) :
    logger{loggerLevel}, threshold{iThreshold} {
  distanceSensor = std::make_unique<pros::Distance>(port);
  if(distanceSensor->is_installed()) {
    logger.debug("Distance sensor found on port " +
                 std::to_string(distanceSensor->get_port()) + ".");
  } else {
    logger.error("Distance sensor at port " +
                 std::to_string(distanceSensor->get_port()) +
                 " could not be initialized!");
  }
}

DistanceSensor::DistanceSensor(const millimeter_t iThreshold,
                               const Logger::Level loggerLevel) :
    logger{loggerLevel}, threshold{iThreshold} {
  const auto distanceSensors{pros::Distance::get_all_devices()};
  if(!distanceSensors.size()) {
    logger.error("Distance sensor not found!");
    return;
  } else if(distanceSensors.size() > 1) {
    logger.warn("Multiple distance sensors found! Using first port found.");
  }
  distanceSensor =
      std::make_unique<pros::Distance>(distanceSensors.front().get_port());
  logger.debug("Distance sensor found on port " +
               std::to_string(distanceSensor->get_port()) + ".");
}

millimeter_t DistanceSensor::getDistance() {
  const int32_t distance{distanceSensor->get_distance()};
  if(distance == noObjectDistance) {
    logger.debug("Distance sensor cannot detect object.");
  }
  return millimeter_t{distance};
}

bool DistanceSensor::closeTo() {
  return getDistance() <= threshold;
}

} // namespace atum