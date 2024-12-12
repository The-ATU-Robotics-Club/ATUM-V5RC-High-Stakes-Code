#pragma once

#include "adi.hpp"

namespace atum {
class LineTracker {
  public:
  LineTracker(const std::uint8_t port);

  LineTracker(const ADIExtenderPort &port);

  private:
  pros::adi::LineSensor lineTracker;
};
} // namespace atum