#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Skills")
  drive->setPose({-2.5_tile, 2.5_tile, 90_deg});
  Trajectory::setDefaultParams(
      {1, 76.5_in_per_s, 76.5_in_per_s_sq, drive->getGeometry().track, 1_in});
  AcceptableDistance acceptable{5_s, 1_in};
  std::unique_ptr<Controller> left{
      std::make_unique<PID>(PID::Parameters{0, 0, 0, 30.5})};
  std::unique_ptr<Controller> right{
      std::make_unique<PID>(PID::Parameters{0, 0, 0, 30.5})};
  std::unique_ptr<PathFollower> pathFollower{std::make_unique<PathFollower>(drive.get(),
                                     acceptable,
                                     std::move(left),
                                     std::move(right),
                                     2.0,
                                     0.7,
                                     Logger::Level::Debug)};
  pathFollower->follow({{-1.5_tile, 1.5_tile, 180_deg}});
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