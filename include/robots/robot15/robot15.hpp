#pragma once

#include "atum/devices/robot.hpp"
#include "atum/devices/remote.hpp"

namespace atum {
class Robot15 : public Robot {
  public:
  friend class Robot;
  Robot15();

  void disabled() override;

  void opcontrol() override;

  private:
  void initializeRoutines() override;
  
  atum::Remote remote{pros::E_CONTROLLER_MASTER};
  pros::MotorGroup leftMotors{1, 2, 3};
  pros::MotorGroup rightMotors{4, 5, 6};
  pros::MotorGroup intake{-7, 8};
};
} // namespace atum