#include "atum/pose/pose.hpp"
#include "atum/systems/robot.hpp"
#include "robotClone.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(RobotClone) {
  START_ROUTINE("Skills")
  END_ROUTINE

  START_ROUTINE("Positive Double Goal")
  setupRoutine({-2_tile - 4_in, -0.5_tile, 90_deg});
  intake->index();
  pathFollower->follow(
      {{{-1.1_tile, -0.5_tile, 90_deg, 76.5_in_per_s},
        false,
        Path::Parameters{1, 76.5_in_per_s, 76.5_in_per_s_sq}},
       {{-0.4_tile, -0.4_tile, 40_deg},
        false,
        Path::Parameters{0.5, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  wait(0.5_s);
  pathFollower->follow(
      {{{-1.1_tile, -1.1_tile, 45_deg},
        true,
        Path::Parameters{1, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  turn->toward({0_tile, -2_tile});
  pathFollower->follow(
      {{{-0.4_tile, -1.6_tile, 135_deg},
        false,
        Path::Parameters{1, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  pathFollower->follow(
      {{{-1.25_tile, -0.75_tile, 135_deg},
        true,
        Path::Parameters{1, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  turn->toward(-45_deg);
  pathFollower->follow(
      {{{-1_tile, -1_tile, -45_deg}, true, Path::Parameters{0.25}}});
  goalClamp->clamp();
  wait(0.1_s);
  intake->intake();
  wait(1_s);
  turn->toward({-1_tile, -2.5_tile});
  pathFollower->follow(
      {{{-1_tile, -2.5_tile, angle(drive->getPose(), {-1_tile, -2.5_tile})},
        false,
        Path::Parameters{2}}});
  turn->toward({-2_tile, -2_tile});
  pathFollower->follow({{{-2_tile,
                          -2_tile,
                          angle(drive->getPose(), {-2_tile, -2_tile})},
                         false,
                         Path::Parameters{1.5}}});
  turn->toward(-135_deg);
  intake->resetCount();
  while(intake->getCount() < 5) {
    pathFollower->follow({{{-2.575_tile, -2.575_tile, -135_deg},
                           false,
                           Path::Parameters{0.125, 15_in_per_s, 20_in_per_s_sq}},
                          {{-2_tile, -2_tile, -135_deg},
                           true,
                           Path::Parameters{0.25, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  }
  intake->stop();
  turn->toward(45_deg);
  pathFollower->follow(
      {{{-2.55_tile, -2.55_tile, 180_deg + angle(drive->getPose(), {-3_tile, -3_tile})},
        true,
        Path::Parameters{0.25}}});
  goalClamp->unclamp();
  pathFollower->follow(
      {{{-0.05_tile, -0.95_tile, 30_deg}, false, Path::Parameters{2.5, 30_in_per_s}}});

  END_ROUTINE

  START_ROUTINE("Full Test")
  setupRoutine({-5_ft, -3_ft, 180_deg});

  pathFollower->follow(
      {{{-1_ft, -4.625_ft, 90_deg}, false, Path::Parameters{2.8, 25_in_per_s}}},
      "Test Curve");
  turn->toward(-15_deg);
  pathFollower->follow(
      {{{0_tile, 0_tile, 45_deg}, false, Path::Parameters{3.5}}});
  turn->awayFrom(135_deg);
  pathFollower->follow(
      {{{1.5_tile, -2.5_tile, 0_deg}, true, Path::Parameters{4}},
       {{1.5_tile, -1.5_tile, 0_deg}, false}});
  turn->toward({0_in, 0_in});

  END_ROUTINE

  START_ROUTINE("Turning Test")
  setupRoutine({0_in, 0_in, 0_deg});

  turn->toward(90_deg);    // Right
  turn->toward(180_deg);   // Right
  turn->toward(-90_deg);   // Right
  turn->awayFrom(0_deg);   // Left
  turn->awayFrom(180_deg); // About
  turn->toward(90_deg);    // Right
  turn->toward(-90_deg);   // About
  turn->awayFrom(0_deg);   // Left
  turn->awayFrom(180_deg); // About
  turn->toward(45_deg);
  turn->toward(90_deg);
  turn->toward(135_deg);
  turn->toward(180_deg);
  turn->toward(225_deg);
  turn->toward(270_deg);
  turn->toward(315_deg);
  turn->toward(360_deg);
  END_ROUTINE

  START_ROUTINE("Ladybrown Test")
  intake->load();
  wait(5_s);
  intake->stop();
  END_ROUTINE

  START_ROUTINE("Pathing Test")
  setupRoutine({-2.5_tile, -1.5_tile, 90_deg});

  Path::Parameters testParams{2, 76.5_in_per_s, 40_in_per_s_sq};
  testParams.usePosition = true;
  pathFollower->follow({{{-0.5_tile, -1.5_tile, 90_deg}, false, testParams}});
  wait(2_s);
  pathFollower->follow({{{-2.5_tile, -1.5_tile, 90_deg}, true, testParams}});
  END_ROUTINE

  START_ROUTINE("Test 3")
  END_ROUTINE
}

void RobotClone::setupRoutine(Pose startingPose) {
  const bool flipped{GUI::Routines::selectedColor() == MatchColor::Blue};
  if(flipped) {
    startingPose.flip();
  }
  pathFollower->setFlipped(flipped);
  turn->setFlipped(flipped);

  drive->setPose(startingPose);
  gps->setPose(startingPose);

  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    intake->setSortOutColor(ColorSensor::Color::Blue);
  } else {
    intake->setSortOutColor(ColorSensor::Color::Red);
  }

  goalClamp->unclamp();
}
} // namespace atum