#pragma once

#include "atum/devices/robot.hpp"
#include "atum/devices/remote.hpp"

namespace atum {
class RobotPrototype : public Robot {
  public:
  friend class Robot;
  RobotPrototype();

  void disabled() override;

  void opcontrol() override;

  private:
  void initializeRoutines() override;

  atum::Remote remote{pros::E_CONTROLLER_MASTER};
  pros::MotorGroup leftMotors{1, 2};
  pros::MotorGroup rightMotors{3, 4};
};
} // namespace atum