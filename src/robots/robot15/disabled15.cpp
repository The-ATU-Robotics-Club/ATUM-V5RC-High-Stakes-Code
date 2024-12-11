#include "robot15.hpp"

namespace atum {
Robot15::Robot15() : Robot{this} {
  ladybrownArm.setBrakeMode(pros::v5::MotorBrake::hold);

  std::unique_ptr<Motor> intakeMtr{
      std::make_unique<Motor>(PortsList{-5, 6}, pros::v5::MotorGears::blue)};
  Intake::Parameters intakeParams;
  intakeParams.jamVelocity = 50_rpm;
  intakeParams.timerUntilJamChecks = Timer{0.1_s};
  intakeParams.timeUntilUnjammed = 0.25_s;
  intake = std::make_unique<Intake>(
      std::move(intakeMtr), intakeParams, Logger::Level::Debug);
}

void Robot15::disabled() {
  intake->startBackgroundTasks();
}
} // namespace atum