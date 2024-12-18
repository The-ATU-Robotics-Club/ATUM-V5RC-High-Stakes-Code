#pragma once

#include "screen.hpp"

namespace atum {
namespace GUI {
class Log : public Screen {
  public:
  friend class Manager;
  static void write(const std::string &msg);

  private:
  static void setupScreen();

  static const std::size_t maxLogLines;
  static lv_obj_t *logTextLabel;
  static std::string logText;
  static std::size_t logLines;
};
} // namespace GUI
} // namespace atum