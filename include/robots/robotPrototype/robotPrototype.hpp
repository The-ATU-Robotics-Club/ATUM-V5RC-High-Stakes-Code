#pragma once

#include "atum/atum.hpp"

namespace atum {
class RobotPrototype : public Robot {
  ROBOT_BOILERPLATE();

  public:
  friend class Robot;
  RobotPrototype();

  void disabled() override;

  void opcontrol() override;

  private:
  Remote remote;
  Motor leftMotors{{1, 2}, Motor::Gearing{pros::v5::MotorGears::green}};
  Motor rightMotors{{3, 4}, Motor::Gearing{pros::v5::MotorGears::green}};
  Motor climbMotors{{9, -10}, Motor::Gearing{pros::v5::MotorGears::green}};
};
} // namespace atum