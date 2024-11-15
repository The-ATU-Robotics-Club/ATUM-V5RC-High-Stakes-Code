#pragma once

#include "atum/devices/robot.hpp"
#include "atum/devices/remote.hpp"

namespace atum {
class RobotPrototype : public Robot {
  public:
  friend class Robot;
  RobotPrototype(std::initializer_list<std::int8_t> leftPorts,
          std::initializer_list<std::int8_t> rightPorts);

  void disabled() override;

  void opcontrol() override;

  private:
  void initializeRoutines() override;

  atum::Remote remote{pros::E_CONTROLLER_MASTER};
  pros::MotorGroup leftMotors;
  pros::MotorGroup rightMotors;
};
} // namespace atum