#pragma once

#include "liblvgl/lvgl.h"
#include <string>
#include <vector>

namespace atum {
namespace GUI {
enum SeriesColor { Red, Green, Blue };

class Screen {
  protected:
  struct GUISize {
    const int width;
    const int height;
  };

  struct GUIPosition {
    const int x;
    const int y;
    const lv_align_t align{LV_ALIGN_TOP_LEFT};
  };

  static void createScreens();

  static void initializeStyles();

  static void addStyles(lv_obj_t *obj,
                        const std::vector<lv_style_t *> &styles,
                        const lv_style_selector_t selector = LV_STATE_DEFAULT);

  static void changeScreenCB(lv_event_t *event);

  static lv_obj_t *createScreenChangeButton(
      lv_obj_t *currentScr,
      const std::string &text,
      const GUISize &size,
      const GUIPosition &position,
      lv_obj_t *nextScr = nullptr,
      const std::vector<lv_style_t *> &extraIdleStyles = {},
      const std::vector<lv_style_t *> &extraPressedStyles = {});

  static lv_obj_t *createLabel(lv_obj_t *scr,
                               const std::string &text,
                               const GUISize &size,
                               const GUIPosition &position,
                               const std::vector<lv_style_t *> &styles = {});

  // Commonly used dimensions/lengths.
  static const int screenWidth;
  static const int screenHeight;
  static const int bgBorderWidth;
  static const int workingWidth;
  static const int workingHeight;
  static const int defaultHeight;
  static const int defaultPadding;
  static const int fillWidth;
  static const int fillHeight;
  static const int contentYOffset;

  // Commonly used colors.
  static const lv_color_t black;
  static const lv_color_t darkGrey;
  static const lv_color_t grey;
  static const lv_color_t lightGrey;
  static const lv_color_t red;
  static const lv_color_t green;
  static const lv_color_t blue;
  static const lv_color_t white;

  // Commonly used styles.
  static lv_style_t styleButtonIdle;
  static lv_style_t styleButtonPressed;
  static lv_style_t styleBG;
  static lv_style_t leftBorder;
  static lv_style_t rightBorder;
  static lv_style_t styleTitle;
  static lv_style_t styleDropDownIdle;
  static lv_style_t styleDropDownPressed;
  static lv_style_t styleChart;

  // Screens.
  static lv_obj_t *loadingScreen;
  static lv_obj_t *mainMenuScreen;
  static lv_obj_t *homeScreen;
  static lv_obj_t *routinesScreen;
  static lv_obj_t *logScreen;
  static lv_obj_t *graphScreen;
  static lv_obj_t *mapScreen;
};
} // namespace GUI
} // namespace atum