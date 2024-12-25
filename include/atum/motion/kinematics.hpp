/**
 * @file kinematics.hpp
 * @brief Includes the Kinematics template class and helpful aliases.
 * @date 2024-12-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "../utility/units.hpp"

namespace atum {
template <typename Unit>
class Kinematics {
  public:
  // Do not allow the construction of an object.
  Kinematics() = delete;
  Kinematics(const Kinematics &) = delete;
  Kinematics(Kinematics &&) = delete;
  
  // Figure out the corresponding types for the derivative of Unit. 
  using UnitsPerSecond = decltype(Unit{1} / 1_s);
  using UnitsPerSecondSq = decltype(UnitsPerSecond{1} / 1_s);
  using UnitsPerSecondCb = decltype(UnitsPerSecondSq{1} / 1_s);

  static Unit position(const second_t dt,
                       const Unit x0,
                       const UnitsPerSecond v0 = UnitsPerSecond{0.0},
                       const UnitsPerSecondSq a0 = UnitsPerSecondSq{0.0},
                       const UnitsPerSecondCb j = UnitsPerSecondCb{0.0}) {
    const auto dt2 = dt * dt;
    return x0 + v0 * dt + 0.5 * a0 * dt2 + (1.0 / 6.0) * j * dt2 * dt;
  }

  static UnitsPerSecond
      velocity(const second_t dt,
               const UnitsPerSecond v0,
               const UnitsPerSecondSq a0 = UnitsPerSecondSq{0.0},
               const UnitsPerSecondCb j = UnitsPerSecondCb{0.0}) {
    return v0 + a0 * dt + 0.5 * j * dt * dt;
  }

  static UnitsPerSecond velocity(const Unit dx,
                                 const UnitsPerSecond v0,
                                 const UnitsPerSecondSq a0 = UnitsPerSecondSq{
                                     0.0}) {
    return v0 + 2.0 * a0 * dx;
  }

  static UnitsPerSecondSq accel(const second_t dt,
                                const UnitsPerSecondSq a0,
                                const UnitsPerSecondCb j = UnitsPerSecondCb{
                                    0.0}) {
    return a0 + j * dt;
  }
};

using LateralKinematics = Kinematics<meter_t>;
using AngularKinematics = Kinematics<radian_t>;
} // namespace atum