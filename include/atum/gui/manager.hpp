#pragma once

#include "graph.hpp"
#include "log.hpp"
#include "map.hpp"
#include "routines.hpp"
#include "screen.hpp"

namespace atum {
namespace GUI {
class Manager : public Screen {
  public:
  static void initialize();

  static void startLoading(const std::string &routines);

  static void finishLoading();

  static void error();

  private:
  static void loadingScreenSetup();
  static void homeScreenSetup();
  static void mainMenuScreenSetup();

  static lv_obj_t *splashImage;
};
} // namespace GUI
} // namespace atum