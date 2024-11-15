#pragma once

#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"
#include <array>
#include <iostream>
#include <random>
#include <string>
#include <vector>

enum class MatchColor { Red, Blue };

class GUI {
  public:
  // These can be moved to private later...
  static const int screenWidth;
  static const int screenHeight;

  static void startLoading(const std::string &routines) {
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

  enum SeriesColor { Red, Green, Blue };

  static void finishLoading() {
    lv_scr_load(homeScreen);
  }

  static std::size_t getRoutineIndex() {
    return lv_dropdown_get_selected(routineSelections);
  }

  static MatchColor getMatchColor() {
    return static_cast<MatchColor>(
        lv_obj_has_state(colorSwitch, LV_STATE_CHECKED));
  }

  static void writeToLog(const std::string &msg) {
    if(!lv_obj_has_state(logSwitch, LV_STATE_CHECKED)) return;
    if(logLines >= maxLogLines)
      logText.erase(0, logText.find('\n') + 1);
    else
      logLines++;
    logText += msg + '\n';
    lv_textarea_set_text(logTextArea, logText.c_str());
  }

  static void addGraphPoint(const double point, const SeriesColor seriesColor) {
    lv_chart_series_t *series{graphSeries[seriesColor]};
    const double seriesRange{graphSeriesRanges[seriesColor]};
    const int adjustedPoint{static_cast<int>(point / seriesRange * graphRange)};
    lv_chart_set_next_value(graphChart, series, adjustedPoint);
    lv_chart_refresh(graphChart);
  }

  static void setGraphSeriesRange(const double range,
                                  const SeriesColor seriesColor) {
    graphSeriesRanges[seriesColor] = range;
  }

  static void clearGraphSeries(const SeriesColor seriesColor) {
    lv_chart_series_t *series{graphSeries[seriesColor]};
    lv_chart_set_all_value(graphChart, series, LV_CHART_POINT_NONE);
  }

  static void addMapPoints(const std::vector<std::pair<int, int>> &points,
                           const SeriesColor seriesColor) {
    lv_chart_series_t *series{mapSeries[seriesColor]};
    for(std::pair<int, int> point : points)
      lv_chart_set_next_value2(mapChart, series, point.first, point.second);
    // Need to manually add last point
    lv_chart_set_next_value2(
        mapChart, series, points.back().first, points.back().second);
    lv_chart_refresh(mapChart);
  }

  static void addMapPoint(const std::pair<int, int> point,
                          const SeriesColor seriesColor) {
    lv_chart_series_t *series{mapSeries[seriesColor]};
    lv_chart_set_next_value2(mapChart, series, point.first, point.second);
    lv_chart_refresh(mapChart);
  }

  static void clearMapSeries(const SeriesColor seriesColor) {
    lv_chart_series_t *series{mapSeries[seriesColor]};
    lv_chart_set_all_value(mapChart, series, LV_CHART_POINT_NONE);
  }

  private:
  // Commonly used dimensions/lengths.
  static const int bgBorderWidth;
  static const int workingWidth;
  static const int workingHeight;
  static const int defaultHeight;
  static const int defaultPadding;
  static const int fillWidth;
  static const int fillHeight;
  static const int contentYOffset;
  static const std::size_t maxLogLines;
  static const int graphRange;
  static const int mapRange;

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

  // Important objects.
  static lv_obj_t *routineSelections;
  static lv_obj_t *colorSwitch;
  static lv_obj_t *logTextArea;
  static lv_obj_t *logSwitch;
  static lv_obj_t *graphChart;
  static std::array<lv_chart_series_t *, 3> graphSeries;
  static std::array<double, 3> graphSeriesRanges;
  static lv_obj_t *mapChart;
  static std::array<lv_chart_series_t *, 3> mapSeries;

  // Other members.
  static std::string logText;
  static std::size_t logLines;

  struct GUISize {
    const int width;
    const int height;
  };

  struct GUIPosition {
    const int x;
    const int y;
    const lv_align_t align{LV_ALIGN_TOP_LEFT};
  };

  static void addStyles(lv_obj_t *obj,
                        const std::vector<lv_style_t *> &styles,
                        const lv_style_selector_t selector = LV_STATE_DEFAULT) {
    for(lv_style_t *style : styles) lv_obj_add_style(obj, style, selector);
  }

  static void changeScreenCB(lv_event_t *event) {
    void *userData{lv_event_get_user_data(event)};
    lv_obj_t *scr{static_cast<lv_obj_t *>(userData)};
    if(scr) lv_scr_load(scr);
  }

  static lv_obj_t *createScreenChangeButton(
      lv_obj_t *currentScr,
      const std::string &text,
      const GUISize &size,
      const GUIPosition &position,
      lv_obj_t *nextScr = nullptr,
      const std::vector<lv_style_t *> &extraIdleStyles = {},
      const std::vector<lv_style_t *> &extraPressedStyles = {}) {
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

  static lv_obj_t *createLabel(lv_obj_t *scr,
                               const std::string &text,
                               const GUISize &size,
                               const GUIPosition &position,
                               const std::vector<lv_style_t *> &styles = {}) {
    lv_obj_t *label{lv_label_create(scr)};
    lv_label_set_text(label, text.c_str());
    lv_obj_set_size(label, size.width, size.height);
    addStyles(label, styles);
    lv_obj_align(label, position.align, position.x, position.y);
    return label;
  }

  static void initializeStyles() {
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
                             static_cast<lv_border_side_t>(
                                 LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_LEFT));
    lv_style_set_border_color(&leftBorder, white);
    lv_style_set_border_width(&leftBorder, 2);
    lv_style_set_radius(&leftBorder, 5);

    // Right border style.
    lv_style_init(&rightBorder);
    lv_style_set_border_side(&rightBorder,
                             static_cast<lv_border_side_t>(
                                 LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT));
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

  static void createScreens() {
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

  static void loadingScreenSetup() {
    lv_obj_t *loadingLabel{lv_label_create(loadingScreen)};
    lv_label_set_text(loadingLabel, "LOADING!   DO NOT TOUCH!");
    lv_obj_center(loadingLabel);
    lv_obj_set_width(loadingLabel, screenWidth);
    lv_label_set_long_mode(loadingLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_color(loadingLabel, white, LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(
        loadingLabel, &lv_font_montserrat_48, LV_STATE_DEFAULT);
  }

  static void homeScreenSetup() {
    lv_obj_t *atumLogo = lv_img_create(homeScreen);
    LV_IMG_DECLARE(atumimg)
    LV_IMG_DECLARE(kelly)

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
    lv_obj_align(
        clickAnywhereLabel, LV_ALIGN_BOTTOM_MID, 0, -2 * defaultPadding);
    lv_obj_set_style_text_color(clickAnywhereLabel, white, LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(
        clickAnywhereLabel, &lv_font_montserrat_12, LV_STATE_DEFAULT);
  }

  static void mainMenuScreenSetup() {
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

  static void routinesScreenSetup(const std::string routines) {
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
    lv_obj_add_style(
        routineSelections, &styleDropDownPressed, LV_STATE_PRESSED);
    lv_obj_add_style(
        routineSelectionsList, &styleDropDownIdle, LV_STATE_DEFAULT);
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

  static void logScreenSetup() {
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
    lv_obj_set_style_bg_color(
        logSwitch, black, LV_PART_KNOB | LV_STATE_CHECKED);

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

  static void graphScreenSetup() {
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

  static void mapScreenSetup() {
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
    lv_obj_align(
        mapChart, LV_ALIGN_TOP_LEFT, 6 * defaultPadding, defaultPadding);
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
};