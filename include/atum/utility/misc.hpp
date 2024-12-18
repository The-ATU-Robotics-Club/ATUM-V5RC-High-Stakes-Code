#pragma once

#include "api.h"
#include "pros/apix.h"
#include <fstream>
#include <numeric>

namespace atum {
/**
 * @brief Because make_unique and make_shared seemed to brick the type inference
 * of constructors involving vectors, this alias makes constructed smart
 * pointers to said objects a little nicer looking.
 *
 *
 */
using PortsList = std::vector<std::int8_t>;

/**
 * @brief A simple templated function to take an average of a vector of items.
 *
 * @tparam T
 * @param items
 * @return T
 */
template <typename T>
T average(const std::vector<T> &items) {
  if(items.empty()) {
    return T{0};
  }
  const double size{static_cast<double>(items.size())};
  return std::reduce(items.begin(), items.end()) / size;
}
} // namespace atum