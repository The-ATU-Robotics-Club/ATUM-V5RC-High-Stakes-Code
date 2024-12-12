#pragma once

#include "../time/time.hpp"
#include "adi.hpp"

namespace atum {
class LineTracker {
  public:
  LineTracker(const std::uint8_t port,
              const std::int32_t iThreshold,
              const Logger::Level loggerLevel = Logger::Level::Info);

  LineTracker(const ADIExtenderPort &port,
              const std::int32_t iThreshold,
              const Logger::Level loggerLevel = Logger::Level::Info);

  bool triggered();

  std::int32_t getReading();

  private:
  void calibrate();

  pros::adi::LineSensor lineTracker;
  const std::int32_t threshold;
  Logger logger;
  static constexpr second_t calibrationTime{500_ms};
};
} // namespace atum