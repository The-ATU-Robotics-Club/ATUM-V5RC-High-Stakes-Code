#pragma once

#include "../time/time.hpp"
#include "../utility/logger.hpp"
#include "../utility/units.hpp"
#include "pros/apix.h"

namespace atum {
/**
 * @brief A class to wrap around several IMUs and provide some useful
 * methods for common actions. Supports dynamically finding port numbers.
 *
 */
class IMU {
  public:
  /**
   * @brief Constructs a new IMU object with port numbers given.
   *
   * @param ports
   * @param iReversed
   * @param loggerLevel
   */
  IMU(std::vector<std::uint8_t> ports,
      const bool iReversed = -1,
      Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Constructs a new IMU object with ports dynamically found.
   * Minimum amount refers to the acceptable minimum amount of IMUs to be
   * found to not trigger a warning (zero IMUs will trigger an error).
   *
   * @param minimumAmount
   * @param iReversed
   * @param loggerLevel
   */
  IMU(const std::size_t minimumAmount,
      const bool iReversed = -1,
      Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Sets the heading of the IMUs.
   *
   * @param heading
   */
  void setHeading(degree_t heading);

  /**
   * @brief Gets the current heading of the IMUs.
   *
   * @return degree_t
   */
  degree_t getHeading();

  /**
   * @brief Gets the angle traveled by the IMUs since this
   * method was last called.
   *
   * @return degree_t
   */
  degree_t getTraveled();

  private:
  /**
   * @brief Helper for the constructor. Sets the data rates of the IMUs,
   * calibrates them, and does some logging.
   *
   */
  void initializeIMUs();

  std::vector<std::unique_ptr<pros::IMU>> imus;
  const bool reversed;
  Logger logger;
  degree_t previous{0_deg};
};
} // namespace atum