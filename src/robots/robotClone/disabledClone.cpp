#include "atum/devices/adi.hpp"
#include "atum/utility/misc.hpp"
#include "robotClone.hpp"

namespace atum {
// White
void RobotClone::initialize15Ports() {
  motorPorts = {{"left drive", {-16, 17, -18, -19}},
                {"right drive", {6, -7, 8, 9}},
                {"ladybrown", {-13}},
                {"intake", {-14, 15}}};
  otherPorts = {{"imu 1", 10},
                {"imu 2", 20},
                {"ladybrown distance", 11},
                {"ladybrown rotation", 12},
                {"intake color 1", 4},
                {"intake color 2", 5},
                {"adi extender", 3},
                {"goal clamp piston", 'F'},
                {"goal limit switch 1", 'A'},
                {"goal limit switch 2", 'B'},
                {"forward odometer 1", 'A'},
                {"forward odometer 2", 'B'},
                {"side odometer 1", 'C'},
                {"side odometer 2", 'D'},
                {"goal rush piston l", 'G'},
                {"goal rush piston r", 'H'},
                {"kaboomer", 'F'}};
}

// Yellow
void RobotClone::initialize24Ports() {
  motorPorts = {{"left drive", {-16, 17, -18, -19}},
                {"right drive", {6, 7, 8, -9}},
                {"ladybrown", {-13}},
                {"intake", {14, -15}}};
  otherPorts = {{"imu 1", 10}, //
                {"imu 2", 20},
                {"ladybrown distance", 11},
                {"ladybrown rotation", 12},
                {"intake color 1", 4},
                {"intake color 2", 5},
                {"adi extender", 3},
                {"goal clamp piston", 'E'},
                {"goal limit switch 1", 'A'},
                {"goal limit switch 2", 'B'},
                {"forward odometer 1", 'C'},
                {"forward odometer 2", 'D'},
                {"side odometer 1", 'A'},
                {"side odometer 2", 'B'},
                {"goal rush piston l", 'H'},
                {"goal rush piston r", 'G'},
                {"kaboomer", 'F'}};
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
  kaboomer = std::make_unique<Piston>(otherPorts["kaboomer"], true, true);
  ladybrown->setIntake(intake.get());
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
                                  Drive::Geometry{11.9_in, 10.21_in});

  const inch_t wheelCircumference{202_mm};
  std::unique_ptr<Odometer> forwardOdometer{
      std::make_unique<Odometer>(otherPorts["forward odometer 1"],
                                 otherPorts["forward odometer 2"],
                                 wheelCircumference,
                                 0.086_in)};
  std::unique_ptr<Odometer> sideOdometer{
      std::make_unique<Odometer>(otherPorts["side odometer 1"],
                                 otherPorts["side odometer 2"],
                                 wheelCircumference,
                                 -1.685_in)};
  std::unique_ptr<IMU> imu{std::make_unique<IMU>(
      PortsList{otherPorts["imu 1"], otherPorts["imu 2"]})};
  std::unique_ptr<Odometry> odometry{std::make_unique<Odometry>(
      std::move(forwardOdometer), std::move(sideOdometer), std::move(imu))};
  odometry->startBackgroundTasks();

  std::unique_ptr<OTOS> otos{
      std::make_unique<OTOS>(21, Pose{0_in, 0_in, -90_deg})};

  const PoseEstimator::StateCovariance P{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.01, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.01, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.02}};

  const PoseEstimator::StateCovariance Q{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 1, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 100, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 20}};

  const PoseEstimator::OutputCovariance R{
      {0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0003, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.00003, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002}};

  std::unique_ptr<PoseEstimator> estimator{std::make_unique<PoseEstimator>(
      drive.get(), std::move(odometry), std::move(otos), P, Q, R)};

  wait(100_ms);
  estimator->startBackgroundTasks();

  drive->setTracker(std::move(estimator));
}

void RobotClone::ladybrownSetup() {
  std::unique_ptr<Motor> ladybrownMotor{
      std::make_unique<Motor>(motorPorts["ladybrown"],
                              Motor::Gearing{pros::v5::MotorGears::green, 3},
                              "ladybrown")};
  std::unique_ptr<DistanceSensor> ladybrownDistanceSensor{
      std::make_unique<DistanceSensor>(otherPorts["ladybrown distance"])};
  std::unique_ptr<RotationSensor> ladybrownRotationSensor{
      std::make_unique<RotationSensor>(otherPorts["ladybrown rotation"],
                                       false)};
  Ladybrown::Parameters ladybrownParameters{
      12.0,
      {5_deg, 235_deg},
      15_deg,
      60_deg,
      0.1_s,
      PID{{0.25, 0.0, 0.05}},
      1,
      SlewRate{std::pair<double, double>{0.8, 0.8}},
      AcceptableAngle{1.5_s, 3_deg, 3_rpm, 0.1_s},
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
      motorPorts["intake"], Motor::Gearing{pros::v5::MotorGears::blue})};
  std::vector<ColorSensor::HueField> hueFields{
      {ColorSensor::Color::Red, 10, 30}, {ColorSensor::Color::Blue, 215, 30}};
  std::unique_ptr<ColorSensor> colorSensor{std::make_unique<ColorSensor>(
      PortsList{otherPorts["intake color 1"], otherPorts["intake color 2"]},
      hueFields)};
  Intake::Parameters intakeParams;
  intakeParams.jamVelocity = 20_rpm;
  intakeParams.timerUntilJamChecks = Timer{0.25_s};
  intakeParams.timeUntilUnjammed = 0.3_s;
  intakeParams.sortThrowTime = 0.05_s;
  intakeParams.pressLoadTime = 300_ms;
  intakeParams.backupFromLoad = 20_deg;
  intakeParams.generalTimeout = 1_s;
  intakeParams.indexingVoltage = 10.0;
  intakeParams.pressVoltage = 7.0;
  intake = std::make_unique<Intake>(std::move(intakeMtr),
                                    std::move(colorSensor),
                                    ladybrown.get(),
                                    intakeParams);
}

void RobotClone::goalSetup() {
  // Setup goal clamp.
  std::unique_ptr<Piston> goalClampPiston{
      std::make_unique<Piston>(otherPorts["goal clamp piston"],
                               id == ID15 ? true : false,
                               id == ID15 ? true : false)};
  std::unique_ptr<LimitSwitch> limitSwitch1{
      std::make_unique<LimitSwitch>(ADIExtenderPort{
          otherPorts["adi extender"], otherPorts["goal limit switch 1"]})};
  std::unique_ptr<LimitSwitch> limitSwitch2{
      std::make_unique<LimitSwitch>(ADIExtenderPort{
          otherPorts["adi extender"], otherPorts["goal limit switch 2"]})};
  goalClamp = std::make_unique<GoalClamp>(std::move(goalClampPiston),
                                          std::move(limitSwitch1),
                                          std::move(limitSwitch2));

  // Setup goal rush.
  goalRushL = std::make_unique<Piston>(otherPorts["goal rush piston l"]);
  goalRushR = std::make_unique<Piston>(otherPorts["goal rush piston r"]);
}

void RobotClone::autonSetup() {
  // Turn setup.
  PID turnPID{PID::Parameters{10.0, 0.5, 70.0, 0, 0.2}};
  AcceptableAngle turnAcceptable{forever, 6_deg, 3_rpm};
  turn = std::make_unique<Turn>(drive.get(), turnPID, turnAcceptable);

  // Move to setup.
  PID directionPID{PID::Parameters{7.5}};
  AcceptableDistance moveToAcceptable{forever, 1.5_in, 1.5_in_per_s};
  PID moveToClosePID{PID::Parameters{45, 2.0, 380, 0.0, 0.05}};
  moveToClose = std::make_unique<MoveTo>(
      drive.get(), turn.get(), moveToClosePID, directionPID, moveToAcceptable);
  PID moveToFarPID{PID::Parameters{20.0, 2.0, 185.0, 0.0, 0.05}};
  moveToFar = std::make_unique<MoveTo>(
      drive.get(), turn.get(), moveToFarPID, directionPID, moveToAcceptable);
  PID moveToRushPID{PID::Parameters{35, 0.0, 340.0}};
  moveToRush = std::make_unique<MoveTo>(
      drive.get(), turn.get(), moveToRushPID, directionPID, moveToAcceptable);

  // Path follower setup.
  meters_per_second_t maxV{76.5_in_per_s};
  meters_per_second_squared_t maxA{153_in_per_s_sq};
  Path::setDefaultParams(
      {1_tile, maxV, maxA, maxA, drive->getGeometry().track});
  AcceptableDistance acceptable{forever};
  PID::Parameters moveToCloseVelocityPIDParams{6, 0, 0, 6};
  moveToCloseVelocityPIDParams.ffScaling = true;
  std::unique_ptr<Controller> forwardController{
      std::make_unique<PID>(moveToCloseVelocityPIDParams)};
  std::unique_ptr<Controller> turnController =
      std::make_unique<PID>(PID::Parameters{7});
  const AccelerationConstants kA{2.5, 1.25};
  pathFollower = std::make_unique<PathFollower>(drive.get(),
                                                acceptable,
                                                std::move(forwardController),
                                                std::move(turnController),
                                                kA,
                                                1_ft,
                                                Logger::Level::Debug);
}
} // namespace atum