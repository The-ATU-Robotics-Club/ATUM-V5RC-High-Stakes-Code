#pragma once

#include "../time/time.hpp"
#include "../utility/units.hpp"
#include "adi.hpp"
#include "api.h"
#include <queue>

namespace atum {
/**
 * @brief
 *
 */
class Potentiometer {
  public:
  Potentiometer(const std::uint8_t port);

  Potentiometer(const ADIExtenderPort &port);

  degree_t getPosition() const;

  degrees_per_second_t getVelocity();

  private:
  pros::adi::Potentiometer pot;
  degree_t previousPosition{0_deg};
  second_t previousTime{0_s};

  static constexpr degree_t potMaxAngle{333_deg};
};
} // namespace atum