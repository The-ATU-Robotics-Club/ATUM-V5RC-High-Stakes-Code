#include "robotClone.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(RobotClone) {
  START_ROUTINE("Skills")
  // Path follower setup.
  Path::setDefaultParams(
      {1.5, 40_in_per_s, 40_in_per_s_sq, drive->getGeometry().track, 1_in});
  AcceptableDistance acceptable{10_s, 1_in};
  PID::Parameters pathFollowerPIDParams{0.031, 0, 0, 0.031};
  pathFollowerPIDParams.ffScaling = true;
  std::unique_ptr<Controller> left{
      std::make_unique<PID>(pathFollowerPIDParams)};
  std::unique_ptr<Controller> right{
      std::make_unique<PID>(pathFollowerPIDParams)};
  std::unique_ptr<PathFollower> pathFollower{
      std::make_unique<PathFollower>(drive.get(),
                                     acceptable,
                                     std::move(left),
                                     std::move(right),
                                     AccelerationConstants{0.0, 1.85},
                                     PathFollower::FeedbackParameters{},
                                     Logger::Level::Debug)};

  // Turn setup.
  AngularProfile::Parameters turnMotionParams{
      720_deg_per_s, 10000_deg_per_s_sq, 10000_deg_per_s_cb};
  turnMotionParams.usePosition = true;
  AngularProfile turnProfile{turnMotionParams, Logger::Level::Debug};
  // Timeout here gets set by the follower, so don't worry about the "forever."
  AcceptableAngle turnAcceptable{forever, 1_deg};
  PID::Parameters turnPIDParams{1.0, 0, 0, 0.85};
  turnPIDParams.ffScaling = true;
  std::unique_ptr<Controller> turnVelocityController =
      std::make_unique<PID>(turnPIDParams);
  const AccelerationConstants kA{0.7, 0.1};
  std::unique_ptr<Controller> turnPositionController =
      std::make_unique<PID>(PID::Parameters{4.0});
  std::unique_ptr<AngularProfileFollower> profileFollower =
      std::make_unique<AngularProfileFollower>(
          turnProfile,
          turnAcceptable,
          std::move(turnVelocityController),
          kA,
          std::move(turnPositionController));
  std::unique_ptr<Turn> turn{std::make_unique<Turn>(
      drive.get(), std::move(profileFollower), Logger::Level::Debug)};

  // Testing setup
  Pose startingPose{-5_ft, -3_ft, 180_deg};
  const bool flipped{GUI::Routines::selectedColor() == MatchColor::Blue};
  if(flipped) {
    startingPose.flip();
  }
  pathFollower->setFlipped(flipped);
  turn->setFlipped(flipped);

  drive->setPose(startingPose);

  // Testing
  pathFollower->follow(
      {{{-3_ft, -5_ft, 90_deg, 40_in_per_s}}, {{-1_ft, -5_ft, 90_deg}}},
      "Test Curve");
  turn->toward(-15_deg);
  pathFollower->follow({{{0_tile, 0_tile, 45_deg}}});
  turn->awayFrom(135_deg);
  pathFollower->follow(
      {{{1.5_tile, -2.5_tile, 0_deg}, true}, {{1.5_tile, -1.5_tile, 0_deg}}});

  END_ROUTINE

  START_ROUTINE("Turning Test")
  drive->setPose({0_tile, 0_tile, 0_deg});

  AngularProfile::Parameters turnMotionParams{
      720_deg_per_s, 10000_deg_per_s_sq, 10000_deg_per_s_cb};
  turnMotionParams.usePosition = true;
  AngularProfile turnProfile{turnMotionParams, Logger::Level::Debug};
  // Timeout here gets set by the follower, so don't worry about the "forever."
  AcceptableAngle turnAcceptable{forever, 1_deg};
  PID::Parameters turnPIDParams{1.0, 0, 0, 0.85};
  turnPIDParams.ffScaling = true;
  std::unique_ptr<Controller> turnVelocityController =
      std::make_unique<PID>(turnPIDParams);
  const AccelerationConstants kA{0.7, 0.1};
  std::unique_ptr<Controller> turnPositionController =
      std::make_unique<PID>(PID::Parameters{4.0});
  std::unique_ptr<AngularProfileFollower> profileFollower =
      std::make_unique<AngularProfileFollower>(
          turnProfile,
          turnAcceptable,
          std::move(turnVelocityController),
          kA,
          std::move(turnPositionController),
          1.1,
          Logger::Level::Debug);
  std::unique_ptr<Turn> turn{std::make_unique<Turn>(
      drive.get(), std::move(profileFollower), Logger::Level::Debug)};

  turn->toward(90_deg);              // Right
  turn->toward(180_deg);             // Right
  turn->toward(-90_deg);             // Right
  turn->awayFrom(0_deg);             // Left
  turn->awayFrom(180_deg);           // About
  turn->toward({1_tile, 0_tile});    // Right
  turn->toward({-1_tile, 0_tile});   // About
  turn->awayFrom({0_tile, 1_tile});  // Left
  turn->awayFrom({0_tile, -1_tile}); // About
  END_ROUTINE

  START_ROUTINE("Ladybrown Test")
  intake->load();
  wait(5_s);
  intake->stop();
  END_ROUTINE

  START_ROUTINE("Pathing Test")
  drive->setPose({-2.5_tile, 2.5_tile, -90_deg});
  Path::setDefaultParams(
      {1, 76.5_in_per_s, 76.5_in_per_s_sq, drive->getGeometry().track, 1_in});
  AcceptableDistance acceptable{10_s, 1_in};
  PID::Parameters pathFollowerPIDParams{0.031, 0, 0, 0.031};
  pathFollowerPIDParams.ffScaling = true;
  std::unique_ptr<Controller> left{
      std::make_unique<PID>(pathFollowerPIDParams)};
  std::unique_ptr<Controller> right{
      std::make_unique<PID>(pathFollowerPIDParams)};
  std::unique_ptr<PathFollower> pathFollower{
      std::make_unique<PathFollower>(drive.get(),
                                     acceptable,
                                     std::move(left),
                                     std::move(right),
                                     AccelerationConstants{0.0, 1.85},
                                     PathFollower::FeedbackParameters{},
                                     Logger::Level::Debug)};
  Path::Parameters testParams{2, 0_in_per_s, 38.25_in_per_s_sq};
  testParams.usePosition = true;
  //   pathFollower->follow({{-0.5_tile, 2.5_tile, 90_deg}, false, testParams});
  //   pathFollower->follow({{-1.5_tile, 1.5_tile, 0_deg}, true, testParams});
  pathFollower->follow(
      {{{-1.5_tile, 1.5_tile, 0_deg, 38.25_in_per_s}, true, testParams},
       {{-0.5_tile, 0.5_tile, -90_deg}, true, testParams}});
  pathFollower->follow(
      {{{-1.5_tile, 1.5_tile, 0_deg, 38.25_in_per_s}, false, testParams},
       {{-2.5_tile, 2.5_tile, -90_deg}, false, testParams}});
  //   pathFollower->follow({{-2.5_tile, 2.5_tile, -90_deg}, false,
  //   testParams});
  END_ROUTINE

  START_ROUTINE("Test 3")
  END_ROUTINE
}
} // namespace atum