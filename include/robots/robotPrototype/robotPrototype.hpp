#pragma once

#include "atum/devices/motor.hpp"
#include "atum/devices/remote.hpp"
#include "atum/devices/robot.hpp"

namespace atum {
class RobotPrototype : public Robot {
  public:
  friend class Robot;
  RobotPrototype();

  void disabled() override;

  void opcontrol() override;

  private:
  void initializeRoutines() override;

  Remote remote{pros::E_CONTROLLER_MASTER};
  Motor leftMotors{{1, 2}, pros::v5::MotorGears::green};
  Motor rightMotors{{3, 4}, pros::v5::MotorGears::green};
};
} // namespace atum