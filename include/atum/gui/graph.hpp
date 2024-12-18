#pragma once

#include "screen.hpp"
#include <array>

namespace atum {
namespace GUI {
/**
 * @brief This class encapsulates the logic and construction of the graphing
 * screen of the GUI. This is helpful for tuning motion profiles, controllers,
 * and general troubleshooting.
 *
 */
class Graph : public Screen {
  public:
  // Friend Manager to give access to screen set up. Not ideal, but
  // straightforward solution.
  friend class Manager;

  /**
   * @brief Adds a data point on one of the grpah series.
   *
   * @param point
   * @param seriesColor
   */
  static void addPoint(const double point, const SeriesColor seriesColor);

  /**
   * @brief Sets the range (negative and positive bounds) for one of the series.
   *
   * @param range
   * @param seriesColor
   */
  static void setSeriesRange(const double range, const SeriesColor seriesColor);

  /**
   * @brief Clears all the values in a given series.
   *
   * @param seriesColor
   */
  static void clearSeries(const SeriesColor seriesColor);

  private:
  /**
   * @brief This deals with setting up the actual screen. Private to force the
   * proper series of steps for setup.
   *
   */
  static void setupScreen();

  static const int graphRange;
  static lv_obj_t *graphChart;
  static std::array<lv_chart_series_t *, 3> graphSeries;
  static std::array<double, 3> graphSeriesRanges;
};
} // namespace GUI
} // namespace atum