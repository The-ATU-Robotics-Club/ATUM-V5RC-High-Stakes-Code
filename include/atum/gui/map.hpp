#pragma once

#include "../position/position.hpp"
#include "screen.hpp"
#include <array>

namespace atum {
namespace GUI {
class Map : public Screen {
  public:
  friend class Manager;
  static void addPositions(const std::vector<Position> &positions,
                        const SeriesColor seriesColor);

  static void addPosition(const Position position, const SeriesColor seriesColor);

  static void clearSeries(const SeriesColor seriesColor);

  private:
  static void setupScreen();

  static const int mapRange;
  static lv_obj_t *mapChart;
  static std::array<lv_chart_series_t *, 3> mapSeries;
};
} // namespace GUI
} // namespace atum