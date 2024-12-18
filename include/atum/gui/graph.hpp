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
   * @brief Packs together into named fields the minimum and maximum values
   * expected when graphing for a series.
   *
   */
  struct SeriesRange {
    double minimum;
    double maximum;
  };

  /**
   * @brief Adds a data point on one of the graph series.
   *
   * @param value
   * @param seriesColor
   */
  static void addValue(double value, const SeriesColor seriesColor);

  /**
   * @brief Sets the range (negative and positive bounds) for one of the series.
   *
   * @param range
   * @param seriesColor
   */
  static void setSeriesRange(const SeriesRange &range,
                             const SeriesColor seriesColor);

  /**
   * @brief Clears all the data points in a given series.
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

  static const int graphResolution;
  static lv_obj_t *graphChart;
  static std::array<lv_chart_series_t *, 7> graphSeries;
  static std::array<SeriesRange, 7> graphSeriesRanges;
};
} // namespace GUI
} // namespace atum