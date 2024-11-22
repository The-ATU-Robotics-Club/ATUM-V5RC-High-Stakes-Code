#pragma once

#include "../utility/logger.hpp"
#include "../utility/units.hpp"
#include "pros/apix.h"

namespace atum {
class IMU {
  public:
  IMU(std::vector<std::uint8_t> ports,
      const bool iReversed = -1,
      Logger::LoggerLevel loggerLevel = Logger::LoggerLevel::Info);

  void setHeading(degree_t heading);

  degree_t getHeading();

  degree_t getTraveled();

  private:
  std::vector<std::unique_ptr<pros::IMU>> imus;
  const bool reversed;
  Logger logger;
  degree_t previous{0_deg};
};
} // namespace atum