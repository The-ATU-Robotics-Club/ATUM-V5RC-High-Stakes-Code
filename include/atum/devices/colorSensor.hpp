#pragma once

#include "../../pros/optical.hpp"
#include "../time/time.hpp"
#include "../utility/logger.hpp"
#include <memory>
#include <vector>

namespace atum {
class ColorSensor {
  public:
  /**
   * @brief This should be used whenever polling the color from these
   * sensors, since it results in the best performance. 
   * 
   */
  static constexpr second_t refreshRate{20_ms};

  /**
   * @brief List of colors that can be recognized. Later this class
   * may make use of templates or a more extensible structure.
   *
   */
  enum class Color { None, Red, Green, Blue };

  /**
   * @brief Given a color, what is its typical hue (center) and the area
   * around that measurement where it is still considered the same color
   * (threshold).
   *
   */
  struct HueField {
    Color color;
    double center;
    double threshold;
  };

  /**
   * @brief Constructs a new ColorSensor with the given port. Hue fields given
   * should form a partition (i.e., they shouldn't overlap; think of centers and
   * threshold as "angles"), otherwise will choose color based on order given
   * when in the threshold for multiple fields.
   *
   * Also turns on LED to max brightness and disables gestures.
   *
   * @param port
   * @param iHueFields
   * @param loggerLevel
   */
  ColorSensor(const std::int8_t port,
              const std::vector<HueField> iHueFields,
              const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Constructs a new ColorSensor by finding the port dynamically. Hue
   * fields given should form a partition (i.e., they shouldn't overlap; think
   * of centers and threshold as "angles"), otherwise will choose color based on
   * order given when in the threshold for multiple fields.
   *
   * Also turns on LED to max brightness and disables gestures.
   *
   * @param iHueFields
   * @param loggerLevel
   */
  ColorSensor(const std::vector<HueField> iHueFields,
              const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Gets the detected color. Will return None if outside of proximity
   * threshold given (unless 0).
   *
   * @return Color
   */
  Color getColor() const;

  /**
   * @brief Returns the raw value for hue given by the sensor and logs the
   * result. Mostly provided for tuning purposes, prefer getColor.
   *
   * @return double
   */
  double getRawHue();

  private:
  /**
   * @brief This is the value get_proximity returns when an object is considered
   * near according to the brain device menu.
   *
   */
  static constexpr int32_t nearProximity{255};

  /**
   * @brief Sets the LED to max brightness and disables gestures.
   *
   */
  void initializeColorSensor();

  std::unique_ptr<pros::v5::Optical> colorSensor;
  std::vector<HueField> hueFields;
  Logger logger;
};

} // namespace atum