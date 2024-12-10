#pragma once

#include "api.h"
#include "pros/apix.h"
#include <fstream>
#include <numeric>

namespace atum {
using PortsList = std::vector<std::int8_t>;

static constexpr int ledRed{0xAA0000};
static constexpr int ledBlue{0x0000BB};

static constexpr int brainScreenWidth{480};
static constexpr int brainScreenHeight{240};

template <typename T>
T average(const std::vector<T> &items) {
  if(items.empty()) return T{0};
  const double size{static_cast<double>(items.size())};
  return std::reduce(items.begin(), items.end()) / size;
}

bool fileExists(const std::string &filename);
} // namespace atum