#include "robot15A.hpp"

namespace atum {
Robot15A::Robot15A() : Robot{this} {
  std::unique_ptr<Motor> leftDriveMtr{
      std::make_unique<Motor>(MotorPortsList{-7, -8, -9, 10},
                              Motor::Gearing{pros::v5::MotorGears::blue},
                              "left drive")};
  std::unique_ptr<Motor> rightDriveMtr{
      std::make_unique<Motor>(MotorPortsList{1, 2, 3, -4},
                              Motor::Gearing{pros::v5::MotorGears::blue},
                              "right drive")};
  const inch_t wheelCircumference{203.724231788_mm};
  std::unique_ptr<Odometer> forwardOdometer{
      std::make_unique<Odometer>('C', 'D', wheelCircumference, -1.8625_in)};
  std::unique_ptr<Odometer> sideOdometer{
      std::make_unique<Odometer>('E', 'F', wheelCircumference, 0.25_in)};
  std::unique_ptr<IMU> imu{std::make_unique<IMU>(PortsList{11, 18})};
  std::unique_ptr<Odometry> odometry{std::make_unique<Odometry>(
      std::move(forwardOdometer), std::move(sideOdometer), std::move(imu))};
  odometry->startBackgroundTasks();
  drive = std::make_unique<Drive>(
      std::move(leftDriveMtr), std::move(rightDriveMtr), std::move(odometry));

  std::unique_ptr<Motor> leftLadybrownMotor{
      std::make_unique<Motor>(MotorPortsList{-15},
                              Motor::Gearing{pros::v5::MotorGears::green, 5},
                              "left ladybrown")};
  std::unique_ptr<Motor> rightLadybrownMotor{
      std::make_unique<Motor>(MotorPortsList{16},
                              Motor::Gearing{pros::v5::MotorGears::green, 5},
                              "right ladybrown")};
  std::unique_ptr<Piston> ladybrownPiston{std::make_unique<Piston>('B', false)};
  std::unique_ptr<RotationSensor> ladybrownRotation{
      std::make_unique<RotationSensor>()};
  std::unique_ptr<LineTracker> ladybrownLineTracker{
      std::make_unique<LineTracker>('H', 2700, Logger::Level::Debug)};
  std::unordered_map<LadybrownState, std::optional<degree_t>>
      ladybrownPositions{{LadybrownState::Resting, -11.1_deg},
                         {LadybrownState::Loading, 16.5_deg},
                         {LadybrownState::Preparing, 60_deg},
                         {LadybrownState::Scoring, 125_deg}};
  Ladybrown::Parameters ladybrownParameters{
      6, -5_deg, 50_deg, ladybrownPositions, 0.375_s};
  ladybrownParameters.kG = 0.2;
  ladybrownParameters.holdController = PID{{0.3}};
  ladybrownParameters.balanceController = PID{{0.2}};
  ladybrownParameters.manualSlew = SlewRate{0.2};
  AngularProfile::Parameters ladybrownMotionParams{
      240_deg_per_s, 10000_deg_per_s_sq, 5000_deg_per_s_cb};
  ladybrownMotionParams.usePosition = true;
  // Timeout here gets set by the follower, so don't worry about the "forever."
  AcceptableAngle ladybrownAcceptable{forever, 3_deg};
  PID::Parameters ladybrownPIDParams{0.15, 0, 0, 1.65};
  ladybrownPIDParams.ffScaling = true;
  std::unique_ptr<Controller> ladybrownVelocityController =
      std::make_unique<PID>(ladybrownPIDParams);
  const AngularProfileFollower::AccelerationConstants kA{0.44, 0.1};
  AngularProfile ladybrownProfile{ladybrownMotionParams};
  std::unique_ptr<Controller> ladybrownPositionController =
      std::make_unique<PID>(PID::Parameters{0.15});
  std::unique_ptr<AngularProfileFollower> profileFollower =
      std::make_unique<AngularProfileFollower>(
          ladybrownProfile,
          ladybrownAcceptable,
          std::move(ladybrownVelocityController),
          kA,
          std::move(ladybrownPositionController),
          1.1,
          Logger::Level::Debug);
  ladybrown = std::make_unique<Ladybrown>(std::move(leftLadybrownMotor),
                                          std::move(rightLadybrownMotor),
                                          std::move(ladybrownPiston),
                                          std::move(ladybrownRotation),
                                          std::move(ladybrownLineTracker),
                                          ladybrownParameters,
                                          std::move(profileFollower));

  std::unique_ptr<Motor> intakeMtr{std::make_unique<Motor>(
      MotorPortsList{-5, 6}, Motor::Gearing{pros::v5::MotorGears::blue})};
  std::vector<ColorSensor::HueField> hueFields{
      {ColorSensor::Color::Red, 10, 30}, {ColorSensor::Color::Blue, 216, 30}};
  std::unique_ptr<ColorSensor> colorSensor{
      std::make_unique<ColorSensor>(17, hueFields)};
  Intake::Parameters intakeParams;
  intakeParams.jamVelocity = 30_rpm;
  intakeParams.timerUntilJamChecks = Timer{0.25_s};
  intakeParams.timeUntilUnjammed = 0.25_s;
  intakeParams.sortThrowTime = 0.05_s;
  intakeParams.finishLoadingTime = 0.0125_s;
  intakeParams.generalTimeout = 1_s;
  intake = std::make_unique<Intake>(std::move(intakeMtr),
                                    std::move(colorSensor),
                                    ladybrown.get(),
                                    intakeParams);
}

void Robot15A::disabled() {
  intake->startBackgroundTasks();
  ladybrown->startBackgroundTasks();
}
} // namespace atum