#pragma once

#include "../../pros/motor_group.hpp"
#include "../utility/logger.hpp"
#include "../utility/units.hpp"

namespace atum {
class Motor {
  public:
  Motor(
      const std::vector<std::int8_t> ports,
      const pros::v5::MotorGears gearset,
      const Logger::LoggerLevel loggerLevel = Logger::LoggerLevel::Info);

  Motor(
      const std::vector<std::int8_t> ports,
      const pros::v5::MotorGears gearset,
      const pros::v5::MotorUnits encoderUnits = pros::v5::MotorUnits::degrees,
      const Logger::LoggerLevel loggerLevel = Logger::LoggerLevel::Info);

  void moveVelocity(const revolutions_per_minute_t velocity);
  void moveVoltage(const double voltage);

  degree_t getPosition() const;
  revolutions_per_minute_t getVelocity() const;
  std::int32_t getTargetVelocity() const;
  std::int32_t getCurrentDraw() const;
  std::int32_t getDirection(const std::uint8_t index = 0) const;
  double getEfficiency() const;
  double getPower() const;
  double getTemperature() const;
  double getTorque() const;
  std::int32_t getVoltage() const;
  std::int32_t getCurrentLimit() const;

  std::int32_t setBrakeMode(const pros::motor_brake_mode_e_t mode) const;
  std::int32_t setCurrentLimit(const std::int32_t limit) const;

  private:
  std::vector<std::int8_t> isOverCurrent() const;
  std::vector<std::int8_t> isOverTemp() const;

  std::vector<std::unique_ptr<pros::Motor>> motors;

  Logger logger;
};
} // namespace atum