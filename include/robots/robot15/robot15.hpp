#pragma once

#include "atum/devices/motor.hpp"
#include "atum/devices/remote.hpp"
#include "atum/devices/robot.hpp"

namespace atum {
class Robot15 : public Robot {
  public:
  friend class Robot;
  Robot15();

  void disabled() override;

  void opcontrol() override;

  private:
  void initializeRoutines() override;

  Remote remote{pros::E_CONTROLLER_MASTER};
  Motor leftMotors{{1, 2, 3}, pros::v5::MotorGears::blue};
  Motor rightMotors{{4, 5, 6}, pros::v5::MotorGears::blue};
  Motor intake{{-7, 8}, pros::v5::MotorGears::blue};
  pros::adi::Pneumatics goalClamp{'A', false};
};
} // namespace atum