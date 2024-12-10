#pragma once

#include "../../pros/motor_group.hpp"
#include "../utility/logger.hpp"
#include "../utility/units.hpp"

namespace atum {
/**
 * @brief A wrapper around potentially several motors to help with
 * common actions (averaging readings) and perform logging.
 *
 */
class Motor {
  public:
  /**
   * @brief The max voltage that can be applied to a motor.
   *
   */
  static constexpr double maxVoltage{12};

  /**
   * @brief Construct a new Motor object. Providing a name will give better
   * logging messages.
   *
   * @param ports
   * @param iGearset
   * @param iName
   * @param loggerLevel
   */
  Motor(const std::vector<std::int8_t> ports,
        const pros::v5::MotorGears iGearset,
        const std::string &iName = "",
        const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Sets the target velocity of the motors.
   *
   * @param velocity
   */
  void moveVelocity(const revolutions_per_minute_t velocity);

  /**
   * @brief Sets the current voltage of the motors.
   *
   * @param voltage
   */
  void moveVoltage(const double voltage);

  /**
   * @brief Stops the motors with the current brake mode.
   *
   */
  void brake();

  /**
   * @brief Get the average position of all the motors.
   *
   * @return degree_t
   */
  degree_t getPosition() const;

  /**
   * @brief Get the average velocity of all the motors.
   *
   * @return revolutions_per_minute_t
   */
  revolutions_per_minute_t getVelocity() const;

  /**
   * @brief Get the target velocity of the motors.
   *
   * @return revolutions_per_minute_t
   */
  revolutions_per_minute_t getTargetVelocity() const;

  /**
   * @brief Get the average current draw of all the motors in mA.
   *
   * @return std::int32_t
   */
  std::int32_t getCurrentDraw() const;

  /**
   * @brief Get the average efficiency of all the motors as a percentage.
   *
   * @return double
   */
  double getEfficiency() const;

  /**
   * @brief Get the average power of all the motors in wattage.
   *
   * @return double
   */
  double getPower() const;

  /**
   * @brief Get the average temperature of all the motors in degrees celsius.
   *
   * @return double
   */
  double getTemperature() const;

  /**
   * @brief Get the average torque of all the motors in Nm.
   *
   * @return double
   */
  double getTorque() const;

  /**
   * @brief Get the voltage of the motors in volts.
   *
   * @return std::int32_t
   */
  std::int32_t getVoltage() const;

  /**
   * @brief Get the current limit of the motors in mA.
   *
   * @return std::int32_t
   */
  std::int32_t getCurrentLimit() const;

  /**
   * @brief Set the brake mode of all the motors.
   *
   * @param mode
   */
  void setBrakeMode(const pros::v5::MotorBrake mode) const;

  /**
   * @brief Set the current limit of all the motors in mA.
   *
   * @param limit
   */
  void setCurrentLimit(const std::int32_t limit) const;

  /**
   * @brief Checks if any motors are uninitialized, too hot, or over current. If
   * they are, logs the issue. Runs whenever there is a command to move and
   * upon.
   *
   */
  void motorCheck();

  private:
  /**
   * @brief Helper for logging, formats the name of the motor if given with
   * the port.
   *
   * @param port
   * @return std::string
   */
  std::string getName(const std::int8_t port);

  const pros::v5::MotorGears gearset;
  const std::string name;
  std::vector<std::unique_ptr<pros::v5::Motor>> motors;
  std::vector<char> enabled; // Avoid vector<bool>. 

  Logger logger;
};
} // namespace atum