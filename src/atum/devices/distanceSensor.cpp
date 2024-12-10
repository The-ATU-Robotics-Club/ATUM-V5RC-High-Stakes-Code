#include "distanceSensor.hpp"

namespace atum {
DistanceSensor::DistanceSensor(const std::int8_t port,
                               const millimeter_t iThreshold,
                               const Logger::Level loggerLevel) :
    logger{loggerLevel}, threshold{iThreshold} {
  pros::v5::Device device{port};
  if(device.is_installed()) {
    distanceSensor = std::make_unique<pros::Distance>(port);
    logger.debug("Distance sensor found on port " +
                 std::to_string(device.get_port()) + ".");
  } else {
    logger.warn("Distance sensor at port " + std::to_string(port) +
                " could not be initialized!");
  }

  logger.info("Distance sensor contructed with port " +
              std::to_string(device.get_port()) + ".");
}

DistanceSensor::DistanceSensor(const millimeter_t iThreshold,
                               const Logger::Level loggerLevel) :
    logger{loggerLevel}, threshold{iThreshold} {
  for(const pros::v5::Device device : pros::v5::Device::get_all_devices()) {
    if(device.get_plugged_type() == pros::v5::DeviceType::distance) {
      distanceSensor = std::make_unique<pros::Distance>(device.get_port());
      logger.debug("Distance sensor found on port " +
                   std::to_string(device.get_port()) + ".");
      break;
    }
  }
  if(!distanceSensor) {
    logger.error("Distance sensor not found!!");
  }
}

millimeter_t DistanceSensor::getDistance() {
  const int32_t distance{distanceSensor->get_distance()};
  if(distance == noObjectDistance) {
    logger.warn("Distance sensor cannot detect object.");
  }
  return millimeter_t{distance};
}

bool DistanceSensor::closeTo() {
  return getDistance() <= threshold;
}

} // namespace atum