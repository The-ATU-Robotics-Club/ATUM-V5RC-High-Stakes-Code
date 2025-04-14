#include "atum/depend/units.h"
#include "atum/motion/motionProfile.hpp"
#include "robotClone.hpp"

namespace atum {

void RobotClone::initialize15Ports() {}

void RobotClone::initialize24Ports() {
  motorPorts = {
      {"left drive", {-6, 7, -8, -9}},
      {"right drive", {-1, 2, 3, 5}},
  };
}

RobotClone::RobotClone(const int iID) : Robot{this}, id{iID} {
  if(id == ID15) {
    initialize15Ports();
  } else if(id == ID24) {
    initialize24Ports();
  }
  driveSetup();
  ladybrownSetup();
  intakeSetup();
  goalSetup();
  autonSetup();
  intake->startBackgroundTasks();
  ladybrown->startBackgroundTasks();
}

void RobotClone::disabled() {
  wait(100_ms);
}

void RobotClone::driveSetup() {
  std::unique_ptr<Motor> leftDriveMtr{std::make_unique<Motor>(
      motorPorts["left drive"],
      Motor::Gearing{pros::v5::MotorGears::blue, 48.0 / 36.0},
      "left drive")};
  std::unique_ptr<Motor> rightDriveMtr{std::make_unique<Motor>(
      motorPorts["right drive"],
      Motor::Gearing{pros::v5::MotorGears::blue, 48.0 / 36.0},
      "right drive")};
  drive = std::make_unique<Drive>(std::move(leftDriveMtr),
                                  std::move(rightDriveMtr),
                                  Drive::Geometry{11.862_in, 10.213335_in});

  const inch_t wheelCircumference{198_mm};
  std::unique_ptr<Odometer> forwardOdometer{
      std::make_unique<Odometer>('C', 'D', wheelCircumference, 0.086_in, true)};
  std::unique_ptr<Odometer> sideOdometer{
      std::make_unique<Odometer>('G', 'H', wheelCircumference, -1.685_in)};
  std::unique_ptr<IMU> imu{std::make_unique<IMU>(PortsList{13, 14})};
  std::unique_ptr<Odometry> odometry{
      std::make_unique<Odometry>(std::move(forwardOdometer),
                                 std::move(sideOdometer),
                                 std::move(imu),
                                 drive.get())};
  odometry->startBackgroundTasks();
  drive->setTracker(std::move(odometry));
}

void RobotClone::ladybrownSetup() {
  std::unique_ptr<Motor> ladybrownMotor{
      std::make_unique<Motor>(MotorPortsList{-20},
                              Motor::Gearing{pros::v5::MotorGears::green, 3},
                              "ladybrown")};
  std::unique_ptr<DistanceSensor> ladybrownDistanceSensor{
      std::make_unique<DistanceSensor>(16)};
  std::unique_ptr<RotationSensor> ladybrownRotationSensor{
      std::make_unique<RotationSensor>(12, false)};
  Ladybrown::Parameters ladybrownParameters{
      12.0,
      {5_deg, 235_deg},
      14_deg,
      PID{{0.25, 0.0, 0.05}},
      1,
      SlewRate{std::pair<double, double>{0.8, 0.8}},
      AcceptableAngle{2_s, 2_deg},
      2_rpm,
      120_mm,
      120_mm};
  ladybrown = std::make_unique<Ladybrown>(std::move(ladybrownMotor),
                                          std::move(ladybrownDistanceSensor),
                                          std::move(ladybrownRotationSensor),
                                          ladybrownParameters);
}

void RobotClone::intakeSetup() {
  std::unique_ptr<Motor> intakeMtr{std::make_unique<Motor>(
      MotorPortsList{-10, 11}, Motor::Gearing{pros::v5::MotorGears::blue})};
  std::vector<ColorSensor::HueField> hueFields{
      {ColorSensor::Color::Red, 10, 30}, {ColorSensor::Color::Blue, 215, 30}};
  std::unique_ptr<ColorSensor> colorSensor{
      std::make_unique<ColorSensor>(PortsList{17, 19}, hueFields)};
  Intake::Parameters intakeParams;
  intakeParams.jamVelocity = 20_rpm;
  intakeParams.timerUntilJamChecks = Timer{0.25_s};
  intakeParams.timeUntilUnjammed = 0.3_s;
  intakeParams.sortThrowTime = 0.05_s;
  intakeParams.pressLoadTime = 750_ms;
  intakeParams.backupFromLoad = 15_deg;
  intakeParams.generalTimeout = 1_s;
  intakeParams.indexingVoltage = 9.0;
  intake = std::make_unique<Intake>(std::move(intakeMtr),
                                    std::move(colorSensor),
                                    ladybrown.get(),
                                    intakeParams);
}

void RobotClone::goalSetup() {
  // Setup goal clamp.
  std::unique_ptr<Piston> goalClampPiston{
      std::make_unique<Piston>('F', false, false)};
  std::unique_ptr<LimitSwitch> limitSwitch1{
      std::make_unique<LimitSwitch>(ADIExtenderPort{18, 'A'})};
  std::unique_ptr<LimitSwitch> limitSwitch2{
      std::make_unique<LimitSwitch>(ADIExtenderPort{18, 'B'})};
  goalClamp = std::make_unique<GoalClamp>(std::move(goalClampPiston),
                                          std::move(limitSwitch1),
                                          std::move(limitSwitch2));
  // Setup goal rush.
  std::unique_ptr<Piston> goalRushArm{std::make_unique<Piston>('B')};
  std::unique_ptr<Piston> goalRushClamp{std::make_unique<Piston>('E')};
  std::unique_ptr<LimitSwitch> limitSwitchRush{
      std::make_unique<LimitSwitch>(ADIExtenderPort{18, 'D'})};
  goalRush = std::make_unique<GoalRush>(std::move(goalRushArm),
                                        std::move(goalRushClamp),
                                        std::move(limitSwitchRush));
}

void RobotClone::autonSetup() {
  meters_per_second_t maxV{76.5_in_per_s};
  meters_per_second_squared_t maxA{153_in_per_s_sq};

  // Turn setup.
  AngularProfile::Parameters turnMotionParams{
      720_deg_per_s, 10000_deg_per_s_sq, 10000_deg_per_s_cb};
  turnMotionParams.usePosition = true;
  AngularProfile turnProfile{turnMotionParams};
  // Timeout here gets set by the follower, so don't worry about the "forever."
  AcceptableAngle turnAcceptable{forever, 3_deg, 1.5_rpm};
  PID::Parameters turnPIDParams{2.5, 0, 0, 0.875};
  turnPIDParams.ffScaling = true;
  std::unique_ptr<Controller> turnVelocityController =
      std::make_unique<PID>(turnPIDParams);
  const AccelerationConstants turnKA{0.75, 0.1};
  std::unique_ptr<Controller> turnPositionController =
      std::make_unique<PID>(PID::Parameters{10.0});
  std::unique_ptr<AngularProfileFollower> angularProfileFollower{
      std::make_unique<AngularProfileFollower>(
          turnProfile,
          turnAcceptable,
          std::move(turnVelocityController),
          turnKA,
          std::move(turnPositionController),
          15_deg)};
  turn = std::make_unique<Turn>(drive.get(), std::move(angularProfileFollower));

  // Move to setup.
  LateralProfile::Parameters moveToMotionParams{maxV, maxA, 612_in_per_s_cb};
  moveToMotionParams.usePosition = true;
  LateralProfile moveToProfile{moveToMotionParams};
  AcceptableDistance moveToAcceptable{forever, 1.5_in, 1.5_in_per_s};
  std::unique_ptr<PID> directionController =
      std::make_unique<PID>(PID::Parameters{0.35});
  PID::Parameters moveToVelocityPIDParams{6, 0, 0, 6};
  moveToVelocityPIDParams.ffScaling = true;
  std::unique_ptr<Controller> moveToVelocityPID{
      std::make_unique<PID>(moveToVelocityPIDParams)};
  const AccelerationConstants kA{2.5, 1.25};
  std::unique_ptr<PID> moveToPositionPID =
      std::make_unique<PID>(PID::Parameters{60});
  std::unique_ptr<LateralProfileFollower> lateralProfileFollower{
      std::make_unique<LateralProfileFollower>(moveToProfile,
                                               moveToAcceptable,
                                               std::move(moveToVelocityPID),
                                               kA,
                                               std::move(moveToPositionPID),
                                               3_in)};
  moveTo = std::make_unique<MoveTo>(drive.get(),
                                    turn.get(),
                                    std::move(lateralProfileFollower),
                                    std::move(directionController));

  // Path follower setup.
  Path::setDefaultParams(
      {1_tile, maxV, maxA, maxA, drive->getGeometry().track});
  AcceptableDistance acceptable{forever};
  std::unique_ptr<Controller> forwardController{
      std::make_unique<PID>(moveToVelocityPIDParams)};
  std::unique_ptr<Controller> turnController =
      std::make_unique<PID>(PID::Parameters{7});
  pathFollower = std::make_unique<PathFollower>(drive.get(),
                                                acceptable,
                                                std::move(forwardController),
                                                std::move(turnController),
                                                kA,
                                                1_ft,
                                                Logger::Level::Debug);
}
} // namespace atum