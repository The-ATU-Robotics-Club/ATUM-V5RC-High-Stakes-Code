#pragma once

#include "../time/time.hpp"
#include "adi.hpp"

namespace atum {
/**
 * @brief A wrapper around the VEX line sensor to support dynamic
 * port initialization and to encapsulate detection logic. WIll also
 * calibrate the sensor, so there should be no major reading changes when this
 * is constructed.
 *
 */
class LineTracker {
  public:
  /**
   * @brief Constructs a new line tracker based on the given port and a
   * threshold for which the sensor will be considered "triggered."
   *
   * @param port
   * @param iThreshold
   * @param loggerLevel
   */
  LineTracker(const std::uint8_t port,
              const std::int32_t iThreshold,
              const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Constructs a new line tracker by detecting the port for the sensor
   * and with a threshold for which the sensor will be considered "triggered."
   *
   * @param port
   * @param iThreshold
   * @param loggerLevel
   */
  LineTracker(const ADIExtenderPort &port,
              const std::int32_t iThreshold,
              const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief This will return if the line tracker reading is below the threshold
   * given for it to be considered "triggered." I.e., when an object or line
   * passes in front of it.
   *
   * @return true
   * @return false
   */
  bool triggered();

  /**
   * @brief Gets the current reading of the line tracker and perform logging.
   * Mostly made public for the purposes of tuning.
   *
   * @return std::int32_t
   */
  std::int32_t getReading();

  private:
  /**
   * @brief Calibrates the line tracker and blocks for an appropriate amount
   * of time for the process to be complete.
   *
   */
  void calibrate();

  pros::adi::LineSensor lineTracker;
  const std::int32_t threshold;
  Logger logger;
};
} // namespace atum