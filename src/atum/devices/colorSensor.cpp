#include "colorSensor.hpp"

namespace atum {

ColorSensor::ColorSensor(const std::int8_t port,
                         const std::vector<HueField> iHueFields,
                         const Logger::Level loggerLevel) :
    hueFields{iHueFields}, logger{loggerLevel} {
  colorSensor = std::make_unique<pros::Optical>(port);
  if(colorSensor->is_installed()) {
    logger.debug("Color sensor found on port " +
                 std::to_string(colorSensor->get_port()) + ".");
  } else {
    logger.error("Color sensor at port " +
                 std::to_string(colorSensor->get_port()) +
                 " could not be initialized!");
  }
  logger.info("Color sensor contructed with port " +
              std::to_string(colorSensor->get_port()) + ".");
  initializeColorSensor();
}

ColorSensor::ColorSensor(const std::vector<HueField> iHueFields,
                         const Logger::Level loggerLevel) :
    hueFields{iHueFields}, logger{loggerLevel} {
  const auto colorSensors{pros::Optical::get_all_devices()};
  if(!colorSensors.size()) {
    logger.error("Color sensor not found!");
    return;
  } else if(colorSensors.size() > 1) {
    logger.warn("Multiple color sensors found! Using first port found.");
  }
  colorSensor =
      std::make_unique<pros::Optical>(colorSensors.front().get_port());
  logger.debug("Color sensor found on port " +
               std::to_string(colorSensor->get_port()) + ".");
  initializeColorSensor();
}

ColorSensor::Color ColorSensor::getColor() const {
  if(colorSensor->get_proximity() < nearProximity) {
    return Color::None;
  }
  for(const HueField &hueField : hueFields) {
    const double reading{colorSensor->get_hue()};
    // Have to account for the "angle wrap" here.
    const double difference{remainder(hueField.center - reading, 360.0)};
    if(std::abs(difference) < hueField.threshold) {
      return hueField.color;
    }
  }
  return Color::None;
}

double ColorSensor::getRawHue() {
  const double reading{colorSensor->get_hue()};
  logger.debug("Color sensor hue reading is " + std::to_string(reading) + ".");
  return reading;
}

void ColorSensor::initializeColorSensor() {
  // The abundance of delays in here is because of a seeming undocumented
  // "delay" needed for many of these values to be set.
  wait(100_ms);
  colorSensor->set_led_pwm(100);
  wait(100_ms);
  colorSensor->set_integration_time(3);
  wait(100_ms);
  colorSensor->disable_gesture();
  wait(100_ms);
}
} // namespace atum