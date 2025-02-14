#include "atum/devices/colorSensor.hpp"
#include "robotClone.hpp"

// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.
static const tile_t pushDoubleStackY{2.4625_tile};

namespace atum {

/*
  _ ___ _ _   ___ _   _ _ _
 / | __( | ) / __| |_(_) | |___
 | |__ \V V  \__ \ / / | | (_-<
 |_|___/     |___/_\_\_|_|_/__/

*/
void RobotClone::skills15() {
  setupRoutine({-2.5_tile, 1.5_tile, 90_deg});
  intake->setSortOutColor(ColorSensor::Color::Blue);
  intake->index();
  moveTo->forward({-0.5_tile, 1.5_tile});
  moveTo->forward({0_tile, 2_tile});
  wait(1_s);
  clampWhenReady();
  moveTo->reverse({-1.5_tile, 2_tile},
        LateralProfile::Parameters{30_in_per_s, 30_in_per_s_sq});
  intake->intake();
  moveTo->forward({-1_tile, 1_tile});
  wait(1_s);
  moveTo->forward({-3_tile, 3_tile},
        LateralProfile::Parameters{50_in_per_s, 50_in_per_s_sq});
  moveTo->reverse({-2.75_tile, 2.75_tile});
  turn->awayFrom(-45_deg);
  goalClamp->unclamp();
  wait(0.5_s);
  moveTo->forward({-.25_tile, 2.5_tile});
  intake->load();
  moveTo->forward({.5_tile, 2.5_tile},
          LateralProfile::Parameters{30_in_per_s, 30_in_per_s_sq});
  wait(1_s);
  moveTo->reverse({0_tile, 2.5_tile});
  turn->toward(0_deg);
  ladybrown->fullyExtend();
  moveTo->forward({0_tile, 2.8_tile},
          LateralProfile::Parameters{15_in_per_s, 15_in_per_s_sq});
  wait(1_s);
  moveTo->reverse({0_tile, 2.5_tile},
          LateralProfile::Parameters{45_in_per_s, 45_in_per_s_sq});
  ladybrown->rest();
  clampWhenReady();
  moveTo->reverse({1.2_tile, .8_tile, 135_deg}, // check
          LateralProfile::Parameters{50_in_per_s, 50_in_per_s_sq});
  wait(1_s);
  intake->intake();
  moveTo->forward({1_tile, 2.2_tile});
  wait(1_s);
  moveTo->forward({2.2_tile, 0.8_tile});
  wait(1_s);
  moveTo->forward({2_tile, 2.2_tile});
  wait(1_s);
  for(int i{0}; i < 2; i++) {
  moveTo->forward({3_tile, 3_tile},
          LateralProfile::Parameters{40_in_per_s, 40_in_per_s_sq});
          wait(.5_s);
  moveTo->reverse({2.5_tile, 2.5_tile});
  wait(.5_s);
  }
  turn->awayFrom(45_deg);
  moveTo->reverse({2.8_tile, 2.8_tile},
          LateralProfile::Parameters{10_in_per_s, 10_in_per_s_sq});
  goalClamp->unclamp();
  wait(.5_s);
  moveTo->forward({2.5_tile, -0.5_tile});
  wait(1_s);
  clampWhenReady();
  moveTo->reverse({1.8_tile, 0.2_tile},
          LateralProfile::Parameters{20_in_per_s, 20_in_per_s_sq});
  moveTo->reverse({1.5_tile, 1.5_tile});
  goalClamp->unclamp();
  ladybrown->fullyExtend();
  goalRush->extendArm();
  moveTo->reverse({1_tile, 1_tile});
  moveTo->reverse({.25_tile, .25_tile},
         LateralProfile::Parameters{38_in_per_s, 38_in_per_s_sq});
         
}

/*
      ___ _ _  _ _   ___ _   _ _ _
     |_  ) | |( | ) / __| |_(_) | |___
      / /|_  _|V V  \__ \ / / | | (_-<
     /___| |_|      |___/_\_\_|_|_/__/

*/
void RobotClone::skills24() {
  setupRoutine({-2.5_tile, 0_tile, 90_deg});
  intake->setSortOutColor(ColorSensor::Color::Blue);
  clampWhenReady();
  intake->index();
  moveTo->forward({-2_tile, 0_tile});
  wait(1_s);
  moveTo->reverse({-2.8_tile, 0_tile});
  intake->intake();
  wait(1_s);
  intake->index();
  moveTo->forward({-2_tile, -2_tile});
  wait(1_s);
  moveTo->reverse({-1_tile, -2_tile},
                  LateralProfile::Parameters{30_in_per_s, 30_in_per_s_sq});
  wait(500_ms);
  intake->intake();
  moveTo->forward({-1_tile, -.75_tile});
  wait(1_s);
  moveTo->forward({0_tile, -2_tile});// x -2 -> 0
  wait(1_s);
  moveTo->forward({-2_tile, -2_tile});
  wait(1_s);
  moveTo->forward({-2.75_tile, -2.75_tile});
  wait(1_s);
  moveTo->reverse({-2_tile, -2_tile});
  moveTo->reverse({-2.75_tile, -2.75_tile});
  goalClamp->unclamp();
  intake->load();
  moveTo->forward({0_tile, -2.5_tile});
  clampWhenReady();
  wait(1_s);
  ladybrown->fullyExtend();
  moveTo->forward({0_tile, -2.75_tile});
  wait(500_ms);
  moveTo->reverse({0_tile, -2.5_tile});
  ladybrown->rest();
  intake->index();
  moveTo->forward({1_tile, -2_tile});
  moveTo->reverse({1_tile, -1_tile},
    LateralProfile::Parameters{30_in_per_s, 30_in_per_s_sq});
  wait(100_ms);
  goalClamp->clamp();
  intake->intake();
  moveTo->forward({2_tile, -1_tile});
  wait(500_ms);
  moveTo->forward({2_tile, -2_tile});
  wait(500_ms);
  for(int i{0}; i < 2; i++) {
  moveTo->forward({2.75_tile, -2.75_tile});//reverse after this?
  moveTo->reverse({2.25_tile, -2.25_tile});
  wait(500_ms);
  }
  moveTo->reverse({2.75_tile, -2.75_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveTo->forward({2_tile, -2_tile});
  ladybrown->fullyExtend();
  goalRush->extendArm();
  moveTo->reverse({.25_tile, -.25_tile},
    LateralProfile::Parameters{38.25_in_per_s, 38.25_in_per_s_sq});
}

ROUTINE_DEFINITIONS_FOR(RobotClone) {
  /*
    ___ _   _ _ _
   / __| |_(_) | |___
   \__ \ / / | | (_-<
   |___/_\_\_|_|_/__/
  */
  START_ROUTINE("Skills")
  if(id == ID15) {
    skills15();
  } else if(id == ID24) {
    skills24();
  }
  END_ROUTINE

  /*
          ___ _    _
    ___  / __(_)__| |___
   |___| \__ \ / _` / -_)
         |___/_\__,_\___|

  */
  START_ROUTINE("Negative Side")
  setupRoutine(
      {-2.5_tile + 7.5_in + (id == ID15 ? 0_in : 7.5_in), 1_tile, 90_deg});
  intake->setSortOutColor(ColorSensor::Color::None);
  goalRush->extendArm();
  goalRush->release();
  goalRushWhenReady();
  intake->index();
  pathFollower->follow(
      {{AcceptableDistance{3_s},
        {-0.45_tile, 1.55_tile, (id == ID15 ? 25_deg : 30_deg)},
        false,
        Path::Parameters{
            1_tile, 0_in_per_s, 0_in_per_s_sq, 76.5_in_per_s_sq}}});
  goalRush->grab();
  wait(200_ms);
  moveTo->reverse({-1.25_tile, 0.75_tile});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(0_deg);
  } else {
    turn->toward(110_deg);
  }
  goalRush->release();
  wait(200_ms);
  intake->stop();
  setSortToOpposite();
  clampWhenReady();
  moveTo->reverse({-0.45_tile, 1.55_tile},
                  LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
  goalRush->retractArm();
  goalClamp->clamp();
  wait(200_ms);

  endOfNegativeRoutines();
  END_ROUTINE

  /*
        _     __  __ _    _
      _| |_  |  \/  (_)__| |
     |_   _| | |\/| | / _` |
       |_|   |_|  |_|_\__,_|

  */
  START_ROUTINE("Positive Mid") setupRoutine(
      {-2.5_tile + 7.5_in + (id == ID15 ? 0_in : 7.5_in), -1_tile, 90_deg});
  intake->setSortOutColor(ColorSensor::Color::None);
  goalRush->extendArm();
  goalRush->release();
  intake->index();
  goalRushWhenReady();
  pathFollower->follow(
      {{AcceptableDistance{3_s},
        {-0.45_tile, -0.45_tile, (id == ID15 ? 25_deg : 30_deg)},
        false,
        Path::Parameters{
            1_tile, 0_in_per_s, 0_in_per_s_sq, 76.5_in_per_s_sq}}});
  goalRush->grab();
  wait(2000_ms);
  moveTo->reverse({-1.25_tile, -1.25_tile});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(0_deg);
  } else {
    turn->toward(110_deg);
  }
  goalRush->release();
  wait(2000_ms);
  intake->stop();
  setSortToOpposite();
  clampWhenReady();
  moveTo->reverse({-0.45_tile, -0.45_tile},
                  LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
  goalRush->retractArm();
  goalClamp->clamp();
  wait(400_ms);

  endOfPositiveRoutines();
  END_ROUTINE

  START_ROUTINE("Do Nothing")
  setupRoutine({});
  END_ROUTINE
}

/*
        ___         _
  ___  | __|_ _  __| |
 |___| | _|| ' \/ _` |
       |___|_||_\__,_|

*/
void RobotClone::endOfNegativeRoutines() {
  moveTo->forward({-1_tile, 1_tile});
  intake->intake();

  moveTo->forward({-1_tile, pushDoubleStackY},
                  LateralProfile::Parameters{45_in_per_s});

  moveTo->reverse({-1_tile, 1_tile});
  turn->awayFrom({0_tile, 0_tile});
  goalClamp->unclamp();
  turn->awayFrom({-2_tile, 0_tile});
  clampWhenReady();
  moveTo->reverse({-1.9_tile, 0.1_tile},
                  LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
  goalClamp->clamp();
  wait(200_ms);

  moveTo->forward({-2_tile, 2_tile}, LateralProfile::Parameters{45_in_per_s});
  wait(1_s);
  intake->stop();
  moveTo->forward({-2.5_tile, 2.5_tile},
                  LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
  intake->intake();
  wait(0.25_s);
  moveTo->forward({-3_tile, 3_tile},
                  LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
  moveTo->reverse({-1.85_tile, 1.85_tile});
  wait(0.15_s);
  for(int i{0}; i < 3; i++) {
    moveTo->forward({-2.5_tile, 2.5_tile},
                    LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
    moveTo->reverse({-1.85_tile, 1.85_tile});
    wait(0.15_s);
  }

  turn->toward(-135_deg);
  pathFollower->follow({{AcceptableDistance{3_s},
                         {-2.45_tile, 0_tile, 180_deg, 45_in_per_s},
                         false,
                         Path::Parameters{2.5_tile, 45_in_per_s}},
                        {AcceptableDistance{3_s},
                         {-2_tile, -2_tile, 135_deg},
                         false,
                         Path::Parameters{2.5_tile, 45_in_per_s}}});
  moveTo->reverse({-2.45_tile, -2.45_tile});
  goalClamp->unclamp();
  intake->stop();
  moveTo->forward({-2_tile, -2_tile});
}

/*
    _     ___         _
  _| |_  | __|_ _  __| |
 |_   _| | _|| ' \/ _` |
   |_|   |___|_||_\__,_|

*/
void RobotClone::endOfPositiveRoutines() {
  moveTo->forward({-1_tile, -1_tile});
  intake->intake();

  moveTo->forward({-1_tile, -pushDoubleStackY},
                  LateralProfile::Parameters{40_in_per_s});

  wait(1_s);
  moveTo->forward({-2_tile, -2_tile}, LateralProfile::Parameters{40_in_per_s});
  wait(1.5_s);

  intake->stop();
  moveTo->forward({-2.5_tile, -2.5_tile},
                  LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
  wait(500_ms);
  intake->intake();
  moveTo->reverse({-1.85_tile, -1.85_tile});
  wait(0.15_s);
  for(int i{0}; i < 4; i++) {
    moveTo->forward({-2.5_tile, -2.5_tile},
                    LateralProfile::Parameters{30_in_per_s, 60_in_per_s_sq});
    moveTo->reverse({-1.85_tile, -1.85_tile});
    wait(0.15_s);
  }
  wait(1_s);
  moveTo->forward({-1_tile, 0_tile}, LateralProfile::Parameters{40.5_in_per_s});
}

void RobotClone::setupRoutine(Pose startingPose) {
  matchTimer.setTime();

  const bool flipped{GUI::Routines::selectedColor() == MatchColor::Blue};
  if(flipped) {
    startingPose.flip();
  }
  Movement::setFlipped(flipped);

  drive->setPose(startingPose);

  setSortToOpposite();

  goalClamp->unclamp();

  if(id == ID15) {
    intake->outtake();
    wait(0.1_s);
    intake->stop();
  }

  drive->setBrakeMode(pros::MotorBrake::brake);
}

void RobotClone::clampWhenReady(const second_t timeout) {
  scheduler.schedule({"Clamp When Ready",
                      [=]() { return goalClamp->hasGoal(); },
                      [=]() {
                        if(goalClamp->isClamped()) {
                          return;
                        }
                        turn->interrupt();
                        moveTo->interrupt();
                        pathFollower->interrupt();
                        goalClamp->clamp();
                      },
                      timeout,
                      Scheduler::doNothing});
}

void RobotClone::goalRushWhenReady(const second_t timeout) {
  scheduler.schedule({"Goal Rush Grab When Ready",
                      [=]() { return goalRush->hasGoal(); },
                      [=]() {
                        if(goalRush->isClamped()) {
                          return;
                        }
                        turn->interrupt();
                        moveTo->interrupt();
                        pathFollower->interrupt();
                        goalRush->grab();
                      },
                      timeout,
                      Scheduler::doNothing});
}

void RobotClone::setSortToOpposite() {
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    intake->setSortOutColor(ColorSensor::Color::Blue);
  } else {
    intake->setSortOutColor(ColorSensor::Color::Red);
  }
}
} // namespace atum
