#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Skills")
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
  START_ROUTINE("Test 1")
  intake->load();
  wait(5_s);
  intake->stop();
  END_ROUTINE
  START_ROUTINE("Test 2")
  END_ROUTINE
  START_ROUTINE("Test 3")
  END_ROUTINE
}
} // namespace atum