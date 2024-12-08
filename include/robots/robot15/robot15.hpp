#pragma once

#include "atum/devices/motor.hpp"
#include "atum/devices/remote.hpp"
#include "atum/devices/robot.hpp"

namespace atum {
class Robot15 : public Robot {
  public:
  Robot15();

  void disabled() override;

  void opcontrol() override;

  private:
  ROBOT_BOILERPLATE();

  Remote remote{pros::E_CONTROLLER_MASTER};
  Motor leftMotors{{-7, -8, -9, 10}, pros::v5::MotorGears::blue};
  Motor rightMotors{{1, 2, 3, -4}, pros::v5::MotorGears::blue};
  Motor intake{{-5, 6}, pros::v5::MotorGears::blue};
  Motor ladybrownArm{{15, -16}, pros::v5::MotorGears::green};
  pros::adi::Pneumatics ladybrownWrist{'B', false};
  pros::adi::Pneumatics goalClamp{'A', false};
};
} // namespace atum