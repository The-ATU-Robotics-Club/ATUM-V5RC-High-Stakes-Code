#include "robot15A.hpp"

namespace atum {
Robot15A::Robot15A() : Robot{this} {
  ladybrownArm.setBrakeMode(pros::v5::MotorBrake::hold);

  std::unique_ptr<Motor> intakeMtr{
      std::make_unique<Motor>(PortsList{-5, 6}, pros::v5::MotorGears::blue)};
  std::vector<ColorSensor::HueField> hueFields{
      {ColorSensor::Color::Red, 10, 30}, {ColorSensor::Color::Blue, 216, 30}};
  std::unique_ptr<ColorSensor> colorSensor{
      std::make_unique<ColorSensor>(hueFields)};
  Intake::Parameters intakeParams;
  intakeParams.jamVelocity = 30_rpm;
  intakeParams.timerUntilJamChecks = Timer{0.25_s};
  intakeParams.timeUntilUnjammed = 0.25_s;
  intakeParams.sortThrowTime = 0.05_s;
  intake = std::make_unique<Intake>(std::move(intakeMtr),
                                    std::move(colorSensor),
                                    intakeParams);

  std::unique_ptr<Motor> leftDriveMtr{std::make_unique<Motor>(
      PortsList{-7, -8, -9, 10}, pros::v5::MotorGears::blue, "left drive")};
  std::unique_ptr<Motor> rightDriveMtr{std::make_unique<Motor>(
      PortsList{1, 2, 3, -4}, pros::v5::MotorGears::blue, "right drive")};
  const inch_t wheelCircumference{203.724231788_mm};
  std::unique_ptr<Odometer> forwardOdometer{
      std::make_unique<Odometer>('C', 'D', wheelCircumference, -1.8625_in)};
  std::unique_ptr<Odometer> sideOdometer{
      std::make_unique<Odometer>('E', 'F', wheelCircumference, 0.25_in)};
  std::unique_ptr<IMU> imu{std::make_unique<IMU>(2)};
  std::unique_ptr<Odometry> odometry{
      std::make_unique<Odometry>(std::move(forwardOdometer),
                                 std::move(sideOdometer),
                                 std::move(imu))};
  odometry->startBackgroundTasks();
  drive = std::make_unique<Drive>(
      std::move(leftDriveMtr), std::move(rightDriveMtr), std::move(odometry));
}

void Robot15A::disabled() {
  intake->startBackgroundTasks();
}
} // namespace atum