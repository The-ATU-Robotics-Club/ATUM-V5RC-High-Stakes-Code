/**
 * @file misc.hpp
 * @brief Includes various functions, aliases, and macros that are useful but
 * don't categorize well.
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "api.h"
#include "pros/apix.h"
#include <fstream>
#include <numeric>

namespace atum {
/**
 * @brief Used internally to automatically make use of different macros
 * depending on number of parameters. Do not use explicity.
 *
 */
#define GET_MACRO(_1, _2, NAME, ...) NAME

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