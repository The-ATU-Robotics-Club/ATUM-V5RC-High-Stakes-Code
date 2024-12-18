#pragma once

#include "screen.hpp"
#include <array>

namespace atum {
enum class MatchColor { Red, Blue };

namespace GUI {
class Routines : public Screen {
  public:
  friend class Manager;
  static std::size_t selectedRoutine();

  static MatchColor selectedColor();

  private:
  static void setupScreen(const std::string routines);

  static lv_obj_t *routineSelections;
  static lv_obj_t *colorSwitch;
};
} // namespace GUI
} // namespace atum