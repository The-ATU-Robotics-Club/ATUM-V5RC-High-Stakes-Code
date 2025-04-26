#pragma once

#include "tracker.hpp"

namespace atum {
class OTOS : public Tracker {
  public:
  OTOS(const std::int8_t port,
       const Pose &offset,
       const Logger::Level loggerLevel = Logger::Level::Info);

  
};
} // namespace atum