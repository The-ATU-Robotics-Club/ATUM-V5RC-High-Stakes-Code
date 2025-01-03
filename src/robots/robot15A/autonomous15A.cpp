#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Skills")
  drive->setPose({-2.5_tile, 2.5_tile, 90_deg});
  Trajectory::setDefaultParams(
      {1, 76.5_in_per_s, 60_in_per_s_sq, drive->getGeometry().track, 1_in});
  AcceptableDistance acceptable{5_s, 1_in};
  PID::Parameters pathFollowerPIDParams{0.0305, 0, 0, 0.0305};
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
                                     2.0,
                                     0.7,
                                     Logger::Level::Debug)};
  Trajectory::Parameters testParams{3};
  testParams.usePosition = true;
  pathFollower->follow({{{-0.5_tile, 0.5_tile, 90_deg}, false, testParams},
                        {{-2.5_tile, 2.5_tile, 90_deg}, true, testParams}});
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