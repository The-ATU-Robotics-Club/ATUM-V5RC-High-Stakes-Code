#pragma once

#include "api.h"
#include "pros/apix.h"
#include <fstream>
#include <numeric>

namespace atum {
using PortsList = std::vector<std::int8_t>;

template <typename T>
T average(const std::vector<T> &items) {
  if(items.empty()) return T{0};
  const double size{static_cast<double>(items.size())};
  return std::reduce(items.begin(), items.end()) / size;
}

} // namespace atum