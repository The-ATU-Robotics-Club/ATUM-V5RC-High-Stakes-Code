#pragma once

#include "../utility/logger.hpp"
#include "../utility/units.hpp"

namespace atum {
class DistanceSensor {
  public:
  /**
   * @brief Constructs a new distance sensor with the given port.
   *
   * @param port
   * @param threshold
   * @param loggerLevel
   */
  DistanceSensor(const std::int8_t port,
                 const inch_t threshold = 0_in,
                 const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Constructs a new distance sensor by finding its port dynamically.
   *
   * @param threshold
   * @param loggerLevel
   */
  DistanceSensor(const inch_t threshold = 0_in,
                 const Logger::Level loggerLevel = Logger::Level::Info);
  
  /**
   * @brief Returns the current reading of the distance sensor.
   *
   * @return inch_t
   */
  inch_t getDistance();

  /**
   * @brief Returns true if detected object is within the given
   * threshold.
   *
   * @return true
   * @return false
   */
  bool closeTo();

  private:
  std::unique_ptr<pros::Distance> distanceSensor;
  Logger logger;
};
} // namespace atum