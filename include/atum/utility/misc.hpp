#pragma once

#include "api.h"
#include "pros/apix.h"
#include <fstream>
#include <numeric>

extern "C" pros::c::v5_device_e_t
    pros::c::registry_get_plugged_type(uint8_t port);

namespace atum {
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