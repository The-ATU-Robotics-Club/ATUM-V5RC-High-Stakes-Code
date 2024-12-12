#include "lineTracker.hpp"

namespace atum {
LineTracker::LineTracker(const std::uint8_t port,
                         const std::int32_t iThreshold,
                         const Logger::Level loggerLevel) :
    lineTracker{port}, threshold{iThreshold}, logger{loggerLevel} {
  calibrate();
  logger.debug("Line tracker on port " +
               std::to_string(std::get<1>(lineTracker.get_port())) +
               " has been constructed.");
}

LineTracker::LineTracker(const ADIExtenderPort &port,
                         const std::int32_t iThreshold,
                         const Logger::Level loggerLevel) :
    lineTracker{port()}, threshold{iThreshold}, logger{loggerLevel} {
  calibrate();
  logger.debug("Line tracker on port " +
               std::to_string(std::get<1>(lineTracker.get_port())) +
               " has been constructed.");
}

bool LineTracker::triggered() {
  return getReading() <= threshold;
}

std::int32_t LineTracker::getReading() {
  const std::int32_t reading{lineTracker.get_value_calibrated()};
  logger.debug("Line tracker on port " +
               std::to_string(std::get<1>(lineTracker.get_port())) +
               " is reading " + std::to_string(reading) + ".");
  return reading;
}

void LineTracker::calibrate() {
  lineTracker.calibrate();
  // Give time to calibrate the sensor to different lighting conditions.
  wait(calibrationTime);
}
} // namespace atum