#pragma once

#include "../sharedSystems/intake.hpp"
#include "atum/atum.hpp"

namespace atum {
class Robot15 : public Robot {
  ROBOT_BOILERPLATE();

  public:
  Robot15();

  void disabled() override;

  void opcontrol() override;

  private:
  Remote remote;
  Motor leftMotors{{-7, -8, -9, 10}, pros::v5::MotorGears::blue, "left drive"};
  Motor rightMotors{{1, 2, 3, -4}, pros::v5::MotorGears::blue, "right drive"};
  std::unique_ptr<Intake> intake;
  Motor ladybrownArm{{15, -16}, pros::v5::MotorGears::green, "ladybrown"};
  pros::adi::Pneumatics ladybrownWrist{'B', false};
  pros::adi::Pneumatics goalClamp{'A', false};
  std::unique_ptr<Odometry> odometry;
};
} // namespace atum