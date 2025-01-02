#include "pose.hpp"

namespace atum {
UnwrappedPose::UnwrappedPose(const double iX,
                             const double iY,
                             const double iH,
                             const double iV,
                             const double iW) :
    x{iX}, y{iY}, h{iH}, v{iV}, w{iW} {}

UnwrappedPose::UnwrappedPose(const Pose &pose) :
    x{getValueAs<meter_t>(pose.x)},
    y{getValueAs<meter_t>(pose.y)},
    h{getValueAs<radian_t>(pose.h)},
    v{getValueAs<meters_per_second_t>(pose.v)},
    w{getValueAs<radians_per_second_t>(pose.w)} {}

UnwrappedPose UnwrappedPose::operator+(const UnwrappedPose &rhs) const {
  return {x + rhs.x, y + rhs.y};
}

UnwrappedPose UnwrappedPose::operator-(const UnwrappedPose &rhs) const {
  return {x - rhs.x, y - rhs.y};
}

UnwrappedPose operator*(const double lhs, const UnwrappedPose &rhs) {
  return {lhs * rhs.x, lhs * rhs.y};
}

UnwrappedPose operator*(const UnwrappedPose &lhs, const double rhs) {
  return {lhs.x * rhs, lhs.y * rhs};
}

double distance(const UnwrappedPose &a, const UnwrappedPose &b) {
  return std::sqrt(std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0));
}

double angle(const UnwrappedPose &state, const UnwrappedPose &reference) {
  const double dx{reference.x - state.x};
  const double dy{reference.y - state.y};
  const double dh{M_PI / 2 - std::atan2(dy, dx)};
  return constrainPI(dh);
}

std::string toString(const UnwrappedPose &pose) {
  return "(" + std::to_string(pose.x) + " m, " + std::to_string(pose.y) +
         " m, " + std::to_string(pose.h) + " rad)";
}

Pose::Pose(const meter_t iX,
           const meter_t iY,
           const radian_t iH,
           const meters_per_second_t iV,
           const radians_per_second_t iW) :
    x{iX}, y{iY}, h{iH}, v{iV}, w{iW} {}

Pose::Pose(const UnwrappedPose &unwrappedPose) :
    x{meter_t{unwrappedPose.x}},
    y{meter_t{unwrappedPose.y}},
    h{radian_t{unwrappedPose.h}},
    v{meters_per_second_t{unwrappedPose.v}},
    w{radians_per_second_t{unwrappedPose.w}} {}

Pose Pose::operator+(const Pose &rhs) const {
  return {x + rhs.x, y + rhs.y};
}

Pose Pose::operator-(const Pose &rhs) const {
  return {x - rhs.x, y - rhs.y};
}

Pose operator*(const double lhs, const Pose &rhs) {
  return {lhs * rhs.x, lhs * rhs.y};
}

Pose operator*(const Pose &lhs, const double rhs) {
  return {lhs.x * rhs, lhs.y * rhs};
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

std::string toString(const Pose &pose) {
  return "(" + to_string(pose.x) + ", " + to_string(pose.y) + ", " +
         to_string(pose.h) + ")";
}
} // namespace atum