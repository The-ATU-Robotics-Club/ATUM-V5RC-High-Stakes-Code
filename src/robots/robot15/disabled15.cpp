#include "robot15.hpp"

namespace atum {
Robot15::Robot15() : Robot{this} {
  ladybrownArm.setBrakeMode(pros::v5::MotorBrake::hold);

  std::unique_ptr<Motor> intakeMtr{
      std::make_unique<Motor>(PortsList{-5, 6}, pros::v5::MotorGears::blue)};
  std::vector<ColorSensor::HueField> hueFields{
      {ColorSensor::Color::Red, 10, 30}, {ColorSensor::Color::Blue, 216, 30}};
  std::unique_ptr<ColorSensor> colorSensor{
      std::make_unique<ColorSensor>(hueFields, Logger::Level::Debug)};
  Intake::Parameters intakeParams;
  intakeParams.jamVelocity = 30_rpm;
  intakeParams.timerUntilJamChecks = Timer{0.25_s};
  intakeParams.timeUntilUnjammed = 0.25_s;
  intakeParams.sortThrowTime = 0.05_s;
  intake = std::make_unique<Intake>(std::move(intakeMtr),
                                    std::move(colorSensor),
                                    intakeParams,
                                    Logger::Level::Debug);
}

void Robot15::disabled() {
  intake->startBackgroundTasks();
}
} // namespace atum