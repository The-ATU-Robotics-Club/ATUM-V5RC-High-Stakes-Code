/**
 * @file pose.hpp
 * @brief Includes the Pose struct as well as corresponding functions.
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "../utility/units.hpp"

namespace atum {
struct Pose; // Forward declaration for UnwrappedPose.

/**
 * @brief This is used to "unwrap" the Pose objects for easier manipulation.
 * Generally, Pose should be used over this.
 *
 * All members should be interpreted as their base metric counterparts (meters,
 * meters per second, radians, etcetera).
 *
 */
struct UnwrappedPose {
  UnwrappedPose(const double iX = 0.0,
                const double iY = 0.0,
                const double iH = 0.0,
                const double iV = 0.0,
                const double iW = 0.0);
  UnwrappedPose(const Pose &pose);
  double x{0.0};
  double y{0.0};
  double h{0.0};
  double v{0.0};
  double w{0.0};
  /**
   * @brief Returns a unwrapped pose where the x and y values are the
   * sums of the left and right sides x and y values.
   *
   * @param rhs
   * @return UnwrappedPose
   */
  UnwrappedPose operator+(const UnwrappedPose &rhs) const;

  /**
   * @brief Returns a unwrapped pose where the x and y values are the
   * differences between the left and right sides x and y values.
   *
   * @param rhs
   * @return UnwrappedPose
   */
  UnwrappedPose operator-(const UnwrappedPose &rhs) const;
};

/**
 * @brief Returns a unwrapped pose with the x and y values of the
 * right side by the left side. Ignores heading.
 *
 * @param lhs
 * @param rhs
 * @return UnwrappedPose
 */
UnwrappedPose operator*(const double lhs, const UnwrappedPose &rhs);

/**
 * @brief Returns a unwrapped pose with the x and y values of the
 * left side by the right side. Ignores heading.
 *
 * @param lhs
 * @param rhs
 * @return UnwrappedPose
 */
UnwrappedPose operator*(const UnwrappedPose &lhs, const double rhs);

/**
 * @brief Gets the distance between two given unwrapped poses.
 *
 * @param a
 * @param b
 * @return double
 */
double distance(const UnwrappedPose &a, const UnwrappedPose &b);

/**
 * @brief Gets the angle between the state and reference state as
 * the inverse tangent of the differences of their y's over the
 * difference of their x's.
 *
 * @param state
 * @param reference
 * @return double
 */
double angle(const UnwrappedPose &state, const UnwrappedPose &reference);

/**
 * @brief Returns a string representation of the given unwrapped pose.
 *
 * @param pose
 * @return std::string
 */
std::string toString(const UnwrappedPose &pose);

/**
 * @brief This is the struct used to represent poses: x and y coordinates
 * with a heading. For reference on the coordinate system we use, see the
 * GPS documentation from VEX. Also included (not normally in pose, but here
 * for convenience) is linear and angular velocity.
 *
 */
struct Pose {
  Pose(const meter_t iX = 0_m,
       const meter_t iY = 0_m,
       const radian_t iH = 0_rad,
       const meters_per_second_t iV = 0_mps,
       const radians_per_second_t iW = 0_rad_per_s);
  Pose(const UnwrappedPose &unwrappedPose);
  meter_t x{0_m};
  meter_t y{0_m};
  radian_t h{0_rad};
  meters_per_second_t v{0_mps};
  radians_per_second_t w{0_rad_per_s};

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
 * @brief Returns a string representation of the given pose.
 *
 * @param pose
 * @return std::string
 */
std::string toString(const Pose &pose);
} // namespace atum