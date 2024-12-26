#include "rotationSensor.hpp"

namespace atum {
RotationSensor::RotationSensor(const std::int8_t port,
                               const bool reversed,
                               const Logger::Level loggerLevel) :
    logger{loggerLevel} {
  rotationSensor = std::make_unique<pros::Rotation>(port);
  if(rotationSensor->is_installed()) {
    logger.debug("Rotation sensor found on port " +
                 std::to_string(rotationSensor->get_port()) + ".");
  } else {
    logger.error("Rotation sensor at port " +
                 std::to_string(rotationSensor->get_port()) +
                 " could not be initialized!");
  }
  initializeRotationSensor(reversed);
}

RotationSensor::RotationSensor(const bool reversed,
                               const Logger::Level loggerLevel) :
    logger{loggerLevel} {
  const auto rotationSensors{pros::Rotation::get_all_devices()};
  if(!rotationSensors.size()) {
    logger.error("Rotation sesnsor not found!");
    return;
  } else if(rotationSensors.size() > 1) {
    logger.warn("Multiple rotation sensors found! Using first port found.");
  }
  rotationSensor =
      std::make_unique<pros::Rotation>(rotationSensors.front().get_port());
  logger.debug("Rotation sensor found on port " +
               std::to_string(rotationSensor->get_port()) + ".");
  initializeRotationSensor(reversed);
}

degree_t RotationSensor::getPosition() {
  const double reading{rotationSensor->get_angle()};
  const degree_t value{reading / 100.0};
  logger.debug("Rotation sensor position is: " + to_string(value));
  return value;
}

degree_t RotationSensor::getDisplacement() {
  const double reading{rotationSensor->get_position()};
  const degree_t value{reading / 100.0};
  logger.debug("Rotation sensor displacement is: " + to_string(value));
  return value;
}

degrees_per_second_t RotationSensor::getVelocity() {
  const double reading{rotationSensor->get_velocity()};
  const degrees_per_second_t value{reading};
  logger.debug("Rotation sensor velocity is: " + to_string(value));
  return value;
}

void RotationSensor::reset() {
  rotationSensor->reset_position();
}

void RotationSensor::initializeRotationSensor(const bool reversed) {
  rotationSensor->set_data_rate(5);
  rotationSensor->set_reversed(reversed);
}
} // namespace atum