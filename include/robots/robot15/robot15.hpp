#pragma once

#include "atum/devices/robot.hpp"
#include "atum/devices/remote.hpp"

namespace atum {
class Robot15 : public Robot {
  public:
  Robot15(std::initializer_list<std::int8_t> leftPorts,
          std::initializer_list<std::int8_t> rightPorts);

  void disabled() override;

  void opcontrol() override;

  void autonomous() override;

  private:
  atum::Remote remote{pros::E_CONTROLLER_MASTER};
  pros::MotorGroup leftMotors;
  pros::MotorGroup rightMotors;
};
} // namespace atum