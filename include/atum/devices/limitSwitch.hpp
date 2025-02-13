/**
 * @file limitSwitch.hpp
 * @brief Includes the LimitSwitch class.
 * @date 2025-01-10
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../utility/logger.hpp"
#include "adi.hpp"

namespace atum {
/**
 * @brief A wrapper around the VEX limit switch to support dynamic
 * port initialization and to provide a way to check the sensor is
 * working at the beginning of a match.
 *
 */
class LimitSwitch {
  public:
  /**
   * @brief Constructs a new limit switch based on the given port.
   *
   * @param port
   * @param loggerLevel
   */
  LimitSwitch(const std::uint8_t port,
              const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Constructs a new limit switch by detecting the port for the sensor.
   *
   * @param port
   * @param loggerLevel
   */
  LimitSwitch(const ADIExtenderPort &port,
              const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Returns if the limit switch is pressed.
   *
   * @return true
   * @return false
   */
  bool isPressed();

  /**
   * @brief Returns if the limit switch has been newly pressed.
   *
   * @return true
   * @return false
   */
  bool isNewlyPressed();

  private:
  pros::adi::DigitalIn limitSwitch;
  Logger logger;
};
} // namespace atum