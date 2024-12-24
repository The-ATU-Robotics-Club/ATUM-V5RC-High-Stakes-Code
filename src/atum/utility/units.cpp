#include "units.hpp"

namespace atum {
template <>
degree_t average<degree_t>(const std::vector<degree_t> &angles) {
  scalar_t sinSum{0};
  scalar_t cosSum{0};
  for(degree_t angle : angles) {
    sinSum += sin(angle);
    cosSum += cos(angle);
  }
  return atan2(sinSum / angles.size(), cosSum / angles.size());
}

degree_t constrain180(const degree_t angle) {
  return degree_t{remainder(getValueAs<degree_t>(angle), 360.0)};
}

meter_t difference(const meter_t left, const meter_t right) {
  return left - right;
}

radian_t difference(const radian_t left, const radian_t right) {
  return constrain180(left - right);
}
} // namespace atum