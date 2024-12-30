#include "lineTracker.hpp"

namespace atum {
LineTracker::LineTracker(const std::uint8_t port,
                         const std::int32_t iThreshold,
                         const Logger::Level loggerLevel) :
    lineTracker{port},
    threshold{iThreshold},
    logger{loggerLevel} {
  initializeLineTracker();
}

LineTracker::LineTracker(const ADIExtenderPort &port,
                         const std::int32_t iThreshold,
                         const Logger::Level loggerLevel) :
    lineTracker{port()},
    threshold{iThreshold},
    logger{loggerLevel} {
  initializeLineTracker();
}

bool LineTracker::triggered() {
  return getReading() <= threshold;
}

std::int32_t LineTracker::getReading() {
  check();
  const std::int32_t reading{lineTracker.get_value_calibrated()};
  logger.debug("Line tracker is reading " + std::to_string(reading) + ".");
  return reading;
}

bool LineTracker::check() {
  // Don't use getReading to avoid infinite loop.
  const std::int32_t reading{lineTracker.get_value_calibrated()};
  const bool normalReading{reading > errorThreshold};
  if(!normalReading) {
    logger.error("Detected issue with line tracker.");
  }
  return normalReading;
}

void LineTracker::initializeLineTracker() {
  lineTracker.calibrate();
  // Give time to calibrate the sensor to different lighting conditions.
  wait(adiCalibrationTime);
  logger.debug("Line tracker has been constructed.");
  check();
}
} // namespace atum