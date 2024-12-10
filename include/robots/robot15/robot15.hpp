#pragma once

#include "atum/atum.hpp"

namespace atum {
class Robot15 : public Robot {
  ROBOT_BOILERPLATE();

  public:
  Robot15();

  void disabled() override;

  void opcontrol() override;

  private:
  Remote remote{Remote::Type::Master, Logger::Level::Debug};
  Motor leftMotors{{-7, -8, -9, 10}, pros::v5::MotorGears::blue};
  Motor rightMotors{{1, 2, 3, -4}, pros::v5::MotorGears::blue};
  Motor intake{{-5, 6}, pros::v5::MotorGears::blue};
  Motor ladybrownArm{{15, -16}, pros::v5::MotorGears::green, "ladybrown", Logger::Level::Debug};
  pros::adi::Pneumatics ladybrownWrist{'B', false};
  pros::adi::Pneumatics goalClamp{'A', false};
};
} // namespace atum