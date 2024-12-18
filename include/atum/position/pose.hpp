#pragma once

#include "../utility/units.hpp"

namespace atum {
/**
 * @brief This is the struct used to represent poses: x and y coordinates
 * with a heading. For reference on the coordinate system we use, see the
 * GPS documentation from VEX.
 *
 */
struct Pose {
  tile_t x{0_tile};
  tile_t y{0_tile};
  degree_t h{0_deg};
  /**
   * @brief Returns a pose where the x and y values are the
   * sums of the left and right sides x and y values.
   *
   * @param rhs
   * @return Pose
   */
  Pose operator+(const Pose &rhs) const;

  /**
   * @brief Returns a pose where the x and y values are the
   * differences between the left and right sides x and y values.
   *
   * @param rhs
   * @return Pose
   */
  Pose operator-(const Pose &rhs) const;
};

/**
 * @brief Returns a pose with the x and y values of the
 * right side by the left side. Ignores heading.
 *
 * @param lhs
 * @param rhs
 * @return Pose
 */
Pose operator*(const double lhs, const Pose &rhs);

/**
 * @brief Returns a pose with the x and y values of the
 * left side by the right side. Ignores heading.
 *
 * @param lhs
 * @param rhs
 * @return Pose
 */
Pose operator*(const Pose &lhs, const double rhs);

/**
 * @brief Returns a string representation of the given pose.
 *
 * @param position
 * @return std::string
 */
std::string toString(const Pose &position);

/**
 * @brief Gets the distance between two given poses.
 *
 * @param a
 * @param b
 * @return tile_t
 */
tile_t distance(const Pose &a, const Pose &b);

/**
 * @brief Gets the angle between the state and reference state as
 * the inverse tangent of the differences of their y's over the
 * difference of their x's.
 *
 * @param state
 * @param reference
 * @return degree_t
 */
degree_t angle(const Pose &state, const Pose &reference);

/**
 * @brief Constrains an angle between -180 and 180 degrees.
 *
 * @param angle
 * @return degree_t
 */
degree_t constrain180(const degree_t angle);
} // namespace atum