#include "robot15A.hpp"

namespace atum {
Robot15A::Robot15A() : Robot{this} {
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
  intakeParams.generalTimeout = 1_s;
  intake = std::make_unique<Intake>(
      std::move(intakeMtr), std::move(colorSensor), intakeParams);

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
  std::unique_ptr<Odometry> odometry{std::make_unique<Odometry>(
      std::move(forwardOdometer), std::move(sideOdometer), std::move(imu))};
  odometry->startBackgroundTasks();
  drive = std::make_unique<Drive>(
      std::move(leftDriveMtr), std::move(rightDriveMtr), std::move(odometry));

  std::unique_ptr<Motor> leftLadybrownMotor{std::make_unique<Motor>(
      PortsList{-15}, pros::v5::MotorGears::green, "left ladybrown")};
  std::unique_ptr<Motor> rightLadybrownMotor{std::make_unique<Motor>(
      PortsList{16}, pros::v5::MotorGears::green, "right ladybrown")};
  std::unique_ptr<Piston> ladybrownPiston{std::make_unique<Piston>('B', false)};
  std::unique_ptr<RotationSensor> ladybrownRotation{
      std::make_unique<RotationSensor>()};
  std::unique_ptr<LineTracker> ladybrownLineTracker{
      std::make_unique<LineTracker>('H', 2685)};
  Ladybrown::Parameters ladybrownParameters{
      12, 10_deg, 30_deg, 60_deg, 135_deg, PID{{0}}, PID{{0.1}}};
  AngularProfile::Parameters ladybrownMotionParams{
      240_deg_per_s, 240_deg_per_s_sq, 480_deg_per_s_cb};
  ladybrownMotionParams.usePosition = true;
  AcceptableAngle ladybrownAcceptable{forever, 2_deg};
  PID::Parameters ladybrownPIDParams{0.1, 0, 0, 1.65};
  ladybrownPIDParams.ffScaling = true;
  std::unique_ptr<Controller> ladybrownVelocityController =
      std::make_unique<PID>(ladybrownPIDParams);
  const AngularProfileFollower::AccelerationConstants kA{0.325, 0.175};
  AngularProfile ladybrownProfile{ladybrownMotionParams, Logger::Level::Debug};
  std::unique_ptr<AngularProfileFollower> profileFollower =
      std::make_unique<AngularProfileFollower>(
          ladybrownProfile,
          ladybrownAcceptable,
          std::move(ladybrownVelocityController),
          kA,
          nullptr,
          Logger::Level::Debug);
  ladybrown = std::make_unique<Ladybrown>(std::move(leftLadybrownMotor),
                                          std::move(rightLadybrownMotor),
                                          std::move(ladybrownPiston),
                                          std::move(ladybrownRotation),
                                          std::move(ladybrownLineTracker),
                                          ladybrownParameters,
                                          std::move(profileFollower));
}

void Robot15A::disabled() {
  intake->startBackgroundTasks();
  ladybrown->startBackgroundTasks();
}
} // namespace atum