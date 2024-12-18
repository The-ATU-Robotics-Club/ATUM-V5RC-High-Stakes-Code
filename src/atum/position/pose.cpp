#include "pose.hpp"

namespace atum {
std::string toString(const Pose &position) {
  return "(" + to_string(position.x) + ", " + to_string(position.y) + ", " +
         to_string(position.h) + ")";
}

Pose Pose::operator+(const Pose &rhs) const {
  return {x + rhs.x, y + rhs.y, h};
}

Pose Pose::operator-(const Pose &rhs) const {
  return {x - rhs.x, y - rhs.y, h};
}

Pose operator*(const double lhs, const Pose &rhs) {
  return {lhs * rhs.x, lhs * rhs.y, rhs.h};
}

Pose operator*(const Pose &lhs, const double rhs) {
  return {lhs.x * rhs, lhs.y * rhs, lhs.h};
}

tile_t distance(const Pose &a, const Pose &b) {
  return sqrt(pow<2>(a.x - b.x) + pow<2>(a.y - b.y));
}

degree_t angle(const Pose &state, const Pose &reference) {
  const inch_t dx{reference.x - state.x};
  const inch_t dy{reference.y - state.y};
  const degree_t dh{90_deg - atan2(dy, dx)};
  return constrain180(dh);
}

degree_t constrain180(const degree_t angle) {
  return degree_t{remainder(getValueAs<degree_t>(angle), 360.0)};
}
} // namespace atum