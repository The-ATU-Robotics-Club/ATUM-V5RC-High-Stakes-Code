#include "robotClone.hpp"


namespace atum {

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
                {"goal clamp piston", 'F'},
                {"adi extender", 18},
                {"goal limit switch 1", 'A'},
                {"goal limit switch 2", 'B'},
                {"forward odometer 1", 'C'},
                {"forward odometer 2", 'D'},
                {"side odometer 1", 'G'},
                {"side odometer 2", 'H'},
                {"goal rush piston 1", 'G'},
                {"goal rush switch 1", 'D'},
                {"goal rush piston 2", 'H'},
                {"goal rush switch 2", 'D'}};
}

void RobotClone::initialize24Ports() {
  motorPorts = {{"left drive", {-6, 7, -8, -9}},
                {"right drive", {-1, 2, 3, 5}},
                {"ladybrown", {-20}},
                {"intake", {-10, 11}}};
  otherPorts = {{"imu 1", 13},
                {"imu 2", 14},
                {"ladybrown distance", 16},
                {"ladybrown rotation", 12},
                {"intake color 1", 17},
                {"intake color 2", errorPort},
                {"goal clamp piston", 'F'},
                {"adi extender", 18},
                {"goal limit switch 1", 'A'},
                {"goal limit switch 2", 'B'},
                {"forward odometer 1", 'C'},
                {"forward odometer 2", 'D'},
                {"side odometer 1", 'G'},
                {"side odometer 2", 'H'},
                {"goal rush piston 1", 'B'},
                {"goal rush switch 1", 'D'},
                {"goal rush piston 2", 'B'},
                {"goal rush switch 2", 'D'}};
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

  const inch_t wheelCircumference{198_mm};
  std::unique_ptr<Odometer> forwardOdometer{
      std::make_unique<Odometer>(otherPorts["forward odometer 1"],
                                 otherPorts["forward odometer 2"],
                                 wheelCircumference,
                                 0.086_in,
                                 true)};
  std::unique_ptr<Odometer> sideOdometer{
      std::make_unique<Odometer>(otherPorts["side odometer 1"],
                                 otherPorts["side odometer 2"],
                                 wheelCircumference,
                                 -1.685_in)};
  std::unique_ptr<IMU> imu{std::make_unique<IMU>(
      PortsList{otherPorts["imu 1"], otherPorts["imu 2"]})};
  std::unique_ptr<Odometry> odometry{
      std::make_unique<Odometry>(std::move(forwardOdometer),
                                 std::move(sideOdometer),
                                 std::move(imu),
                                 drive.get())};
  //   odometry->startBackgroundTasks();

  const PoseEstimator::StateCovariance P{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.02, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.05}};

  const PoseEstimator::StateCovariance Q{P};

  const PoseEstimator::OutputCovariance R{{0.1, 0.0}, {0.0, 0.1}};

  std::unique_ptr<PoseEstimator> estimator{
      std::make_unique<PoseEstimator>(drive.get(), P, Q, R)};
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
      16_deg,
      0.0_s,
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
  intakeParams.backupFromLoad = 6_deg;
  intakeParams.generalTimeout = 1_s;
  intake = std::make_unique<Intake>(std::move(intakeMtr),
                                    std::move(colorSensor),
                                    ladybrown.get(),
                                    intakeParams);
}

void RobotClone::goalSetup() {
  // Setup goal clamp.
  std::unique_ptr<Piston> goalClampPiston{
      std::make_unique<Piston>(otherPorts["goal clamp piston"], true, true)};
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
  std::unique_ptr<Piston> goalRushArm1{
      std::make_unique<Piston>(otherPorts["goal rush piston 1"])};
  std::unique_ptr<LimitSwitch> goalRushSwitch1{
      std::make_unique<LimitSwitch>(ADIExtenderPort{
          otherPorts["adi extender"], otherPorts["goal rush switch 1"]})};
  goalRush1 = std::make_unique<GoalRush>(std::move(goalRushArm1),
                                         std::move(goalRushSwitch1));
  std::unique_ptr<Piston> goalRushArm2{
      std::make_unique<Piston>(otherPorts["goal rush piston 2"])};
  std::unique_ptr<LimitSwitch> goalRushSwitch2{
      std::make_unique<LimitSwitch>(ADIExtenderPort{
          otherPorts["adi extender"], otherPorts["goal rush switch 2"]})};
  goalRush2 = std::make_unique<GoalRush>(std::move(goalRushArm2),
                                         std::move(goalRushSwitch2));
}

void RobotClone::autonSetup() {
  meters_per_second_t maxV{76.5_in_per_s};
  meters_per_second_squared_t maxA{153_in_per_s_sq};

  // Turn setup.
  AngularProfile::Parameters turnMotionParams{
      720_deg_per_s, 10000_deg_per_s_sq, 10000_deg_per_s_cb};
  turnMotionParams.usePosition = true;
  AngularProfile turnProfile{turnMotionParams};
  // Timeout here gets set by the follower, so don't worry about the
  // "forever."
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