#pragma once

#include "screen.hpp"
#include <array>

namespace atum {
namespace GUI {
class Graph : public Screen {
  public:
  friend class Manager;
  static void addPoint(const double point, const SeriesColor seriesColor);

  static void setSeriesRange(const double range, const SeriesColor seriesColor);

  static void clearSeries(const SeriesColor seriesColor);

  private:
  static void setupScreen();

  static const int graphRange;
  static lv_obj_t *graphChart;
  static std::array<lv_chart_series_t *, 3> graphSeries;
  static std::array<double, 3> graphSeriesRanges;
};
} // namespace GUI
} // namespace atum