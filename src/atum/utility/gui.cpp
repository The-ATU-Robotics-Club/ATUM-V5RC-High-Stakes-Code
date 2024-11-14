#include "gui.hpp"

// Commonly used dimensions/lengths.
const int GUI::screenWidth{ 480 };
const int GUI::screenHeight{ 240 };
const int GUI::bgBorderWidth{ 5 };
const int GUI::workingWidth{ screenWidth - bgBorderWidth };
const int GUI::workingHeight{ screenHeight - bgBorderWidth };
const int GUI::defaultHeight{ 40 };
const int GUI::defaultPadding{ bgBorderWidth };
const int GUI::fillWidth{ workingWidth - 4 * defaultPadding };
const int GUI::fillHeight{ workingHeight - defaultHeight - 4 * defaultPadding };
const int GUI::contentYOffset{ defaultHeight + 2 * defaultPadding };
const int GUI::graphRange{ 10000 };
const int GUI::mapRange{ 12000 };

// Commonly used colors.
const lv_color_t GUI::black{ lv_color_hex(0x1f1f1f) };
const lv_color_t GUI::darkGrey{ lv_color_hex(0x303030) };
const lv_color_t GUI::grey{ lv_color_hex(0x454545) };
const lv_color_t GUI::lightGrey{ lv_color_hex(0xbebebe) };
const lv_color_t GUI::red{ lv_color_hex(0xFF0000) };
const lv_color_t GUI::green{ lv_color_hex(0x00FF00) };
const lv_color_t GUI::blue{ lv_color_hex(0x0000FF) };
const lv_color_t GUI::white{ lv_color_white() };


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
const std::size_t GUI::maxLogLines{ 25 };

// Screens.
lv_obj_t* GUI::loadingScreen;
lv_obj_t* GUI::mainMenuScreen;
lv_obj_t* GUI::homeScreen;
lv_obj_t* GUI::routinesScreen;
lv_obj_t* GUI::logScreen;
lv_obj_t* GUI::graphScreen;
lv_obj_t* GUI::mapScreen;

// Important objects
lv_obj_t* GUI::routineSelections;
lv_obj_t* GUI::colorSwitch;
lv_obj_t* GUI::logTextArea;
lv_obj_t* GUI::logSwitch;
lv_obj_t* GUI::graphChart;
std::array<lv_chart_series_t*, 3> GUI::graphSeries;
std::array<double, 3> GUI::graphSeriesRanges{ graphRange, graphRange, graphRange };
lv_obj_t* GUI::mapChart;
std::array<lv_chart_series_t*, 3> GUI::mapSeries;

// Other members.
std::string GUI::logText;
std::size_t GUI::logLines;