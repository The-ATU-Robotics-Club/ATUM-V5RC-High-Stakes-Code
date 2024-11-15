#include "gui.hpp"

LV_IMG_DECLARE(atumimg)
LV_IMG_DECLARE(kelly)

namespace atum {
// Commonly used dimensions/lengths.
const int GUI::screenWidth{480};
const int GUI::screenHeight{240};
const int GUI::bgBorderWidth{5};
const int GUI::workingWidth{screenWidth - bgBorderWidth};
const int GUI::workingHeight{screenHeight - bgBorderWidth};
const int GUI::defaultHeight{40};
const int GUI::defaultPadding{bgBorderWidth};
const int GUI::fillWidth{workingWidth - 4 * defaultPadding};
const int GUI::fillHeight{workingHeight - defaultHeight - 4 * defaultPadding};
const int GUI::contentYOffset{defaultHeight + 2 * defaultPadding};
const int GUI::graphRange{10000};
const int GUI::mapRange{12000};

// Commonly used colors.
const lv_color_t GUI::black{lv_color_hex(0x1f1f1f)};
const lv_color_t GUI::darkGrey{lv_color_hex(0x303030)};
const lv_color_t GUI::grey{lv_color_hex(0x454545)};
const lv_color_t GUI::lightGrey{lv_color_hex(0xbebebe)};
const lv_color_t GUI::red{lv_color_hex(0xFF0000)};
const lv_color_t GUI::green{lv_color_hex(0x00FF00)};
const lv_color_t GUI::blue{lv_color_hex(0x0000FF)};
const lv_color_t GUI::white{lv_color_white()};

void GUI::startLoading(const std::string &routines) {
  // Initialization.
  createScreens();
  initializeStyles();

  // Screen setup.
  loadingScreenSetup();
  homeScreenSetup();
  mainMenuScreenSetup();
  routinesScreenSetup(routines);
  logScreenSetup();
  graphScreenSetup();
  mapScreenSetup();

  // Go to the loading screen until given signal robot is good to go.
  lv_scr_load(loadingScreen);
}

void GUI::finishLoading() {
  lv_scr_load(homeScreen);
}

std::size_t GUI::getRoutineIndex() {
  return lv_dropdown_get_selected(routineSelections);
}

MatchColor GUI::getMatchColor() {
  return static_cast<MatchColor>(
      lv_obj_has_state(colorSwitch, LV_STATE_CHECKED));
}

void GUI::writeToLog(const std::string &msg) {
  if(!lv_obj_has_state(logSwitch, LV_STATE_CHECKED)) return;
  if(logLines >= maxLogLines)
    logText.erase(0, logText.find('\n') + 1);
  else
    logLines++;
  logText += msg + '\n';
  lv_textarea_set_text(logTextArea, logText.c_str());
}

void GUI::addGraphPoint(const double point, const SeriesColor seriesColor) {
  lv_chart_series_t *series{graphSeries[seriesColor]};
  const double seriesRange{graphSeriesRanges[seriesColor]};
  const int adjustedPoint{static_cast<int>(point / seriesRange * graphRange)};
  lv_chart_set_next_value(graphChart, series, adjustedPoint);
  lv_chart_refresh(graphChart);
}

void GUI::setGraphSeriesRange(const double range,
                              const SeriesColor seriesColor) {
  graphSeriesRanges[seriesColor] = range;
}

void GUI::clearGraphSeries(const SeriesColor seriesColor) {
  lv_chart_series_t *series{graphSeries[seriesColor]};
  lv_chart_set_all_value(graphChart, series, LV_CHART_POINT_NONE);
}

void GUI::addMapPoints(const std::vector<std::pair<int, int>> &points,
                       const SeriesColor seriesColor) {
  lv_chart_series_t *series{mapSeries[seriesColor]};
  for(std::pair<int, int> point : points)
    lv_chart_set_next_value2(mapChart, series, point.first, point.second);
  // Need to manually add last point
  lv_chart_set_next_value2(
      mapChart, series, points.back().first, points.back().second);
  lv_chart_refresh(mapChart);
}

void GUI::addMapPoint(const std::pair<int, int> point,
                      const SeriesColor seriesColor) {
  lv_chart_series_t *series{mapSeries[seriesColor]};
  lv_chart_set_next_value2(mapChart, series, point.first, point.second);
  lv_chart_refresh(mapChart);
}

void GUI::clearMapSeries(const SeriesColor seriesColor) {
  lv_chart_series_t *series{mapSeries[seriesColor]};
  lv_chart_set_all_value(mapChart, series, LV_CHART_POINT_NONE);
}

void GUI::addStyles(lv_obj_t *obj,
                    const std::vector<lv_style_t *> &styles,
                    const lv_style_selector_t selector) {
  for(lv_style_t *style : styles) lv_obj_add_style(obj, style, selector);
}

void GUI::changeScreenCB(lv_event_t *event) {
  void *userData{lv_event_get_user_data(event)};
  lv_obj_t *scr{static_cast<lv_obj_t *>(userData)};
  if(scr) lv_scr_load(scr);
}

lv_obj_t *GUI::createScreenChangeButton(
    lv_obj_t *currentScr,
    const std::string &text,
    const GUISize &size,
    const GUIPosition &position,
    lv_obj_t *nextScr,
    const std::vector<lv_style_t *> &extraIdleStyles,
    const std::vector<lv_style_t *> &extraPressedStyles) {
  // Create the button, size it, and place it.
  lv_obj_t *button{lv_btn_create(currentScr)};
  lv_obj_set_size(button, size.width, size.height);
  lv_obj_align(button, position.align, position.x, position.y);

  // Add and place the text on the button.
  lv_obj_t *label{lv_label_create(button)};
  lv_label_set_text(label, text.c_str());
  lv_obj_center(label);

  // Add screen change callback with next screen as a parameter.
  lv_obj_add_event_cb(
      button, changeScreenCB, LV_EVENT_CLICKED, static_cast<void *>(nextScr));

  // Add styles.
  lv_obj_add_style(button, &styleButtonIdle, 0);
  addStyles(button, extraIdleStyles);
  lv_obj_add_style(button, &styleButtonPressed, LV_STATE_PRESSED);
  addStyles(button, extraPressedStyles, LV_STATE_PRESSED);

  return button;
};

lv_obj_t *GUI::createLabel(lv_obj_t *scr,
                           const std::string &text,
                           const GUISize &size,
                           const GUIPosition &position,
                           const std::vector<lv_style_t *> &styles) {
  lv_obj_t *label{lv_label_create(scr)};
  lv_label_set_text(label, text.c_str());
  lv_obj_set_size(label, size.width, size.height);
  addStyles(label, styles);
  lv_obj_align(label, position.align, position.x, position.y);
  return label;
}

void GUI::initializeStyles() {
  // Background style.
  lv_style_init(&styleBG);
  lv_style_set_bg_color(&styleBG, grey);
  lv_style_set_bg_opa(&styleBG, LV_OPA_COVER);
  lv_style_set_border_opa(&styleBG, LV_OPA_COVER);
  lv_style_set_border_width(&styleBG, bgBorderWidth);
  lv_style_set_border_color(&styleBG, darkGrey);

  // Left border style.
  lv_style_init(&leftBorder);
  lv_style_set_border_side(&leftBorder,
                           static_cast<lv_border_side_t>(LV_BORDER_SIDE_BOTTOM |
                                                         LV_BORDER_SIDE_LEFT));
  lv_style_set_border_color(&leftBorder, white);
  lv_style_set_border_width(&leftBorder, 2);
  lv_style_set_radius(&leftBorder, 5);

  // Right border style.
  lv_style_init(&rightBorder);
  lv_style_set_border_side(&rightBorder,
                           static_cast<lv_border_side_t>(LV_BORDER_SIDE_BOTTOM |
                                                         LV_BORDER_SIDE_RIGHT));
  lv_style_set_border_color(&rightBorder, white);
  lv_style_set_border_width(&rightBorder, 2);
  lv_style_set_radius(&rightBorder, 5);

  // Title label style.
  lv_style_init(&styleTitle);
  lv_style_set_text_color(&styleTitle, black);
  lv_style_set_text_font(&styleTitle, &lv_font_montserrat_36);
  lv_style_set_text_align(&styleTitle, LV_TEXT_ALIGN_CENTER);
  lv_style_set_text_letter_space(&styleTitle, -4);
  lv_style_set_bg_color(&styleTitle, lightGrey);
  lv_style_set_bg_opa(&styleTitle, LV_OPA_COVER);

  // Idle button style.
  lv_style_init(&styleButtonIdle);
  lv_style_set_text_color(&styleButtonIdle, black);
  lv_style_set_text_font(&styleButtonIdle, &lv_font_montserrat_36);
  lv_style_set_text_letter_space(&styleButtonIdle, -4);
  lv_style_set_bg_color(&styleButtonIdle, lightGrey);
  lv_style_set_bg_opa(&styleButtonIdle, LV_OPA_COVER);
  lv_style_set_shadow_opa(&styleButtonIdle, LV_OPA_TRANSP);

  // Pressed button style.
  lv_style_init(&styleButtonPressed);
  lv_style_set_text_color(&styleButtonPressed, white);
  lv_style_set_bg_color(&styleButtonPressed, black);

  // Idle drop down style
  lv_style_init(&styleDropDownIdle);
  lv_style_set_text_color(&styleDropDownIdle, black);
  lv_style_set_text_font(&styleDropDownIdle, &lv_font_montserrat_20);
  lv_style_set_text_align(&styleDropDownIdle, LV_TEXT_ALIGN_CENTER);
  lv_style_set_bg_color(&styleDropDownIdle, lightGrey);
  lv_style_set_bg_opa(&styleDropDownIdle, LV_OPA_COVER);

  // Pressed drop down style.
  lv_style_init(&styleDropDownPressed);
  lv_style_set_text_color(&styleDropDownPressed, white);
  lv_style_set_bg_color(&styleDropDownPressed, black);

  // Chart style.
  lv_style_init(&styleChart);
  lv_style_set_bg_color(&styleChart, black);
}

void GUI::createScreens() {
  loadingScreen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(loadingScreen,
                            lv_color_black(),
                            LV_PART_MAIN); // Darker black than normal.
  mainMenuScreen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(loadingScreen, black, LV_PART_MAIN);
  homeScreen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(homeScreen,
                            lv_color_black(),
                            LV_PART_MAIN); // Darker black than normal.
  lv_obj_add_event_cb(homeScreen,
                      changeScreenCB,
                      LV_EVENT_CLICKED,
                      static_cast<void *>(mainMenuScreen));
  routinesScreen = lv_obj_create(nullptr);
  lv_obj_add_style(routinesScreen, &styleBG, 0);
  logScreen = lv_obj_create(nullptr);
  lv_obj_add_style(logScreen, &styleBG, 0);
  graphScreen = lv_obj_create(nullptr);
  lv_obj_add_style(graphScreen, &styleBG, 0);
  mapScreen = lv_obj_create(nullptr);
  lv_obj_add_style(mapScreen, &styleBG, 0);
}

void GUI::loadingScreenSetup() {
  lv_obj_t *loadingLabel{lv_label_create(loadingScreen)};
  lv_label_set_text(loadingLabel, "LOADING!   DO NOT TOUCH!");
  lv_obj_center(loadingLabel);
  lv_obj_set_width(loadingLabel, screenWidth);
  lv_label_set_long_mode(loadingLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_obj_set_style_text_color(loadingLabel, white, LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(
      loadingLabel, &lv_font_montserrat_48, LV_STATE_DEFAULT);
}

void GUI::homeScreenSetup() {
  lv_obj_t *atumLogo = lv_img_create(homeScreen);

  srand(pros::micros());
  const int rng = rand() % 15 + 1;
  std::cout << rng << '\n';
  if(rng == 1) {
    lv_img_set_src(atumLogo, &kelly); // Easter egg Randy Kelly!
  } else {
    lv_img_set_src(atumLogo, &atumimg);
  }

  lv_obj_set_style_opa(atumLogo, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_center(atumLogo);
  lv_obj_align(atumLogo, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_size(atumLogo, 427, 240);

  lv_obj_t *clickAnywhereLabel{lv_label_create(homeScreen)};
  lv_label_set_text(clickAnywhereLabel, "Click anywhere to continue.");
  lv_obj_center(clickAnywhereLabel);
  lv_obj_align(clickAnywhereLabel, LV_ALIGN_BOTTOM_MID, 0, -2 * defaultPadding);
  lv_obj_set_style_text_color(clickAnywhereLabel, white, LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(
      clickAnywhereLabel, &lv_font_montserrat_12, LV_STATE_DEFAULT);
}

void GUI::mainMenuScreenSetup() {
  createLabel(mainMenuScreen,
              "MAIN MENU",
              {250, defaultHeight},
              {defaultPadding, defaultPadding},
              {&rightBorder, &styleTitle});

  createScreenChangeButton(
      mainMenuScreen,
      LV_SYMBOL_NEW_LINE,
      {150, defaultHeight},
      {-defaultPadding, defaultPadding, LV_ALIGN_TOP_RIGHT},
      homeScreen,
      {&leftBorder});

  createScreenChangeButton(mainMenuScreen,
                           "ROUTINE",
                           {fillWidth, defaultHeight},
                           {0, contentYOffset, LV_ALIGN_TOP_MID},
                           routinesScreen);

  createScreenChangeButton(
      mainMenuScreen,
      "LOG",
      {fillWidth, defaultHeight},
      {0, contentYOffset + defaultHeight + defaultPadding, LV_ALIGN_TOP_MID},
      logScreen);

  createScreenChangeButton(
      mainMenuScreen,
      "GRAPH",
      {fillWidth, defaultHeight},
      {0,
       contentYOffset + 2 * (defaultHeight + defaultPadding),
       LV_ALIGN_TOP_MID},
      graphScreen);

  createScreenChangeButton(
      mainMenuScreen,
      "MAP",
      {fillWidth, defaultHeight},
      {0,
       contentYOffset + 3 * (defaultHeight + defaultPadding),
       LV_ALIGN_TOP_MID},
      mapScreen);
}

void GUI::routinesScreenSetup(const std::string routines) {
  createLabel(routinesScreen,
              "ROUTINES",
              {250, defaultHeight},
              {defaultPadding, defaultPadding},
              {&rightBorder, &styleTitle});

  createScreenChangeButton(
      routinesScreen,
      LV_SYMBOL_NEW_LINE,
      {150, defaultHeight},
      {-defaultPadding, defaultPadding, LV_ALIGN_TOP_RIGHT},
      mainMenuScreen,
      {&leftBorder});

  routineSelections = lv_dropdown_create(routinesScreen);
  lv_dropdown_set_options(routineSelections, routines.c_str());
  lv_obj_align(routineSelections, LV_ALIGN_TOP_MID, 0, contentYOffset);
  lv_obj_set_size(routineSelections, fillWidth, defaultHeight);
  lv_obj_t *routineSelectionsList{lv_dropdown_get_list(routineSelections)};
  lv_obj_set_style_max_height(
      routineSelectionsList, fillHeight - defaultHeight, LV_STATE_DEFAULT);
  lv_obj_add_style(routineSelections, &styleDropDownIdle, LV_STATE_DEFAULT);
  lv_obj_add_style(routineSelections, &styleDropDownPressed, LV_STATE_PRESSED);
  lv_obj_add_style(routineSelectionsList, &styleDropDownIdle, LV_STATE_DEFAULT);
  lv_obj_add_style(routineSelectionsList,
                   &styleDropDownPressed,
                   LV_PART_SELECTED | LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(
      routineSelectionsList, darkGrey, LV_PART_SELECTED | LV_STATE_CHECKED);

  colorSwitch = lv_switch_create(routinesScreen);
  lv_obj_set_size(colorSwitch, fillWidth, 3 * defaultHeight);
  lv_obj_align(colorSwitch, LV_ALIGN_BOTTOM_MID, 0, -defaultPadding);
  lv_obj_set_style_bg_color(colorSwitch, red, LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(colorSwitch, white, LV_PART_KNOB);
  lv_obj_set_style_bg_color(
      colorSwitch, blue, LV_PART_INDICATOR | LV_STATE_CHECKED);
}

void GUI::logScreenSetup() {
  createLabel(logScreen,
              "LOG",
              {250, defaultHeight},
              {defaultPadding, defaultPadding},
              {&rightBorder, &styleTitle});

  createScreenChangeButton(
      logScreen,
      LV_SYMBOL_NEW_LINE,
      {150, defaultHeight},
      {-defaultPadding, defaultPadding, LV_ALIGN_TOP_RIGHT},
      mainMenuScreen,
      {&leftBorder});

  lv_obj_t *logEnabledLabel{lv_label_create(logScreen)};
  lv_label_set_text(logEnabledLabel, "Logging enabled:");
  lv_obj_center(logEnabledLabel);
  lv_obj_align(logEnabledLabel,
               LV_ALIGN_BOTTOM_RIGHT,
               -defaultPadding - 125,
               -3 * defaultPadding);
  lv_obj_set_style_text_color(logEnabledLabel, white, LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(
      logEnabledLabel, &lv_font_montserrat_18, LV_STATE_DEFAULT);

  logSwitch = lv_switch_create(logScreen);
  lv_obj_set_size(logSwitch, 100, defaultHeight);
  lv_obj_add_state(logSwitch, LV_STATE_CHECKED);
  lv_obj_align(
      logSwitch, LV_ALIGN_BOTTOM_RIGHT, -defaultPadding, -defaultPadding);
  lv_obj_set_style_bg_color(logSwitch, black, LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(logSwitch, white, LV_PART_KNOB);
  lv_obj_set_style_bg_color(
      logSwitch, white, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(logSwitch, black, LV_PART_KNOB | LV_STATE_CHECKED);

  logTextArea = lv_textarea_create(logScreen);
  lv_obj_align(logTextArea, LV_ALIGN_TOP_MID, 0, contentYOffset);
  lv_obj_set_size(
      logTextArea, fillWidth, fillHeight - defaultHeight - defaultPadding);
  lv_obj_set_style_bg_color(logTextArea, lightGrey, LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(logTextArea, LV_OPA_COVER, LV_STATE_DEFAULT);
  lv_obj_set_style_text_color(logTextArea, black, LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(
      logTextArea, white, LV_STATE_DEFAULT | LV_PART_SCROLLBAR);
  lv_textarea_set_max_length(logTextArea, 1000);
}

void GUI::graphScreenSetup() {
  createLabel(graphScreen,
              "GRAPH",
              {250, defaultHeight},
              {defaultPadding, defaultPadding},
              {&rightBorder, &styleTitle});

  createScreenChangeButton(
      graphScreen,
      LV_SYMBOL_NEW_LINE,
      {150, defaultHeight},
      {-defaultPadding, defaultPadding, LV_ALIGN_TOP_RIGHT},
      mainMenuScreen,
      {&leftBorder});

  graphChart = lv_chart_create(graphScreen);
  lv_chart_set_type(graphChart, LV_CHART_TYPE_LINE);
  lv_obj_set_size(graphChart, fillWidth, fillHeight);
  lv_obj_align(graphChart, LV_ALIGN_TOP_MID, 0, contentYOffset);
  lv_chart_set_range(
      graphChart, LV_CHART_AXIS_PRIMARY_Y, -graphRange, graphRange);
  lv_chart_set_div_line_count(graphChart, 9, 18);
  lv_obj_add_style(graphChart, &styleChart, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_chart_set_point_count(graphChart, 1000);

  graphSeries[SeriesColor::Red] =
      lv_chart_add_series(graphChart, red, LV_CHART_AXIS_PRIMARY_Y);
  graphSeries[SeriesColor::Green] =
      lv_chart_add_series(graphChart, green, LV_CHART_AXIS_PRIMARY_Y);
  graphSeries[SeriesColor::Blue] =
      lv_chart_add_series(graphChart, blue, LV_CHART_AXIS_PRIMARY_Y);
}

void GUI::mapScreenSetup() {
  lv_obj_t *mapLabel{
      createLabel(mapScreen,
                  "MAP",
                  {150, defaultHeight},
                  {-6 * defaultPadding, -70, LV_ALIGN_BOTTOM_RIGHT},
                  {&leftBorder, &styleTitle})};
  lv_obj_set_style_border_side(
      mapLabel,
      static_cast<lv_border_side_t>(LV_BORDER_SIDE_TOP | LV_BORDER_SIDE_LEFT),
      LV_STATE_DEFAULT);

  createScreenChangeButton(mapScreen,
                           LV_SYMBOL_NEW_LINE,
                           {150, defaultHeight},
                           {-6 * defaultPadding, 70, LV_ALIGN_TOP_RIGHT},
                           mainMenuScreen,
                           {&leftBorder});

  mapChart = lv_chart_create(mapScreen);
  lv_chart_set_type(mapChart, LV_CHART_TYPE_SCATTER);
  static const int mapChartSize{screenHeight - 4 * defaultPadding};
  lv_obj_set_size(mapChart, mapChartSize, mapChartSize);
  lv_obj_align(mapChart, LV_ALIGN_TOP_LEFT, 6 * defaultPadding, defaultPadding);
  lv_chart_set_range(mapChart, LV_CHART_AXIS_PRIMARY_X, -mapRange, mapRange);
  lv_chart_set_range(mapChart, LV_CHART_AXIS_PRIMARY_Y, -mapRange, mapRange);
  lv_chart_set_div_line_count(mapChart, 9, 9);
  lv_obj_set_style_line_width(mapChart, 0, LV_PART_ITEMS);
  lv_obj_add_style(mapChart, &styleChart, LV_PART_MAIN | LV_STATE_DEFAULT);

  mapSeries[SeriesColor::Red] =
      lv_chart_add_series(mapChart, red, LV_CHART_AXIS_PRIMARY_Y);
  mapSeries[SeriesColor::Green] =
      lv_chart_add_series(mapChart, green, LV_CHART_AXIS_PRIMARY_Y);
  mapSeries[SeriesColor::Blue] =
      lv_chart_add_series(mapChart, blue, LV_CHART_AXIS_PRIMARY_Y);
}

// Commonly used styles.
lv_style_t GUI::styleButtonIdle;
lv_style_t GUI::styleButtonPressed;
lv_style_t GUI::styleBG;
lv_style_t GUI::leftBorder;
lv_style_t GUI::rightBorder;
lv_style_t GUI::styleTitle;
lv_style_t GUI::styleDropDownIdle;
lv_style_t GUI::styleDropDownPressed;
lv_style_t GUI::styleChart;
const std::size_t GUI::maxLogLines{25};

// Screens.
lv_obj_t *GUI::loadingScreen;
lv_obj_t *GUI::mainMenuScreen;
lv_obj_t *GUI::homeScreen;
lv_obj_t *GUI::routinesScreen;
lv_obj_t *GUI::logScreen;
lv_obj_t *GUI::graphScreen;
lv_obj_t *GUI::mapScreen;

// Important objects
lv_obj_t *GUI::routineSelections;
lv_obj_t *GUI::colorSwitch;
lv_obj_t *GUI::logTextArea;
lv_obj_t *GUI::logSwitch;
lv_obj_t *GUI::graphChart;
std::array<lv_chart_series_t *, 3> GUI::graphSeries;
std::array<double, 3> GUI::graphSeriesRanges{graphRange,
                                             graphRange,
                                             graphRange};
lv_obj_t *GUI::mapChart;
std::array<lv_chart_series_t *, 3> GUI::mapSeries;

// Other members.
std::string GUI::logText;
std::size_t GUI::logLines;
} // namespace atum