#include "graph.hpp"

namespace atum {
namespace GUI {
const int Graph::graphRange{10000};

void Graph::addPoint(const double point, const SeriesColor seriesColor) {
  lv_chart_series_t *series{graphSeries[seriesColor]};
  const double seriesRange{graphSeriesRanges[seriesColor]};
  const int adjustedPoint{static_cast<int>(point / seriesRange * graphRange)};
  lv_chart_set_next_value(graphChart, series, adjustedPoint);
  lv_chart_refresh(graphChart);
}

void Graph::setSeriesRange(const double range, const SeriesColor seriesColor) {
  graphSeriesRanges[seriesColor] = range;
}

void Graph::clearSeries(const SeriesColor seriesColor) {
  lv_chart_series_t *series{graphSeries[seriesColor]};
  lv_chart_set_all_value(graphChart, series, LV_CHART_POINT_NONE);
}

void Graph::setupScreen() {
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

lv_obj_t *Graph::graphChart;
std::array<lv_chart_series_t *, 3> Graph::graphSeries;
std::array<double, 3> Graph::graphSeriesRanges{graphRange,
                                               graphRange,
                                               graphRange};
} // namespace GUI
} // namespace atum