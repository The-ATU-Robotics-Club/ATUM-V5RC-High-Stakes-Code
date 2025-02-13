#include "atum/devices/colorSensor.hpp"
#include "robotClone.hpp"

namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.
ROUTINE_DEFINITIONS_FOR(RobotClone) {
  /*
    ___ _   _ _ _
   / __| |_(_) | |___
   \__ \ / / | | (_-<
   |___/_\_\_|_|_/__/
  */
  START_ROUTINE("Skills")

  if(id == ID15) {
    /*
      _ ___ _ _   ___ _   _ _ _
     / | __( | ) / __| |_(_) | |___
     | |__ \V V  \__ \ / / | | (_-<
     |_|___/     |___/_\_\_|_|_/__/

    */
   setupRoutine(
    { 2.5_tile, 0_tile, 270_deg});
    intake->setSortOutColor(ColorSensor::Color::None);
    goalRushWhenReady();
    intake->index();
    moveTo->forward({2_tile, 0_tile});
    moveTo->reverse({2.5_tile, 0_tile});
    intake->intake();
    wait(1000_ms);
    turn->toward(-25_deg);
    moveTo->forward({2_tile, 2_tile});
    intake->index();
    wait(100_ms);
    turn->toward(90_deg);
    moveTo->reverse({1_tile, 2_tile},
                    LateralProfile::Parameters{30_in_per_s, 30_in_per_s_sq});
    wait(100_ms);
    goalClamp->clamp();
    wait(100_ms);
    turn->toward(180_deg);
    intake->intake(); 
    moveTo->forward({1_tile, 1_tile}); 
    wait(1000_ms);
    turn->toward(-45_deg);
    intake->intake();
    moveTo->forward({0_tile, 2.25_tile});
    wait(1000_ms);
    turn->toward(90_deg);
    moveTo->forward({2_tile, 2_tile});
    turn->toward(45_deg);
    wait(100_ms);
    intake->intake();
    moveTo->forward({3_tile, 2.75_tile});
    wait(1000_ms);
    moveTo->reverse({2_tile, 2_tile});
    turn->toward(225_deg);
    moveTo->reverse({2.65_tile, 2.65_tile});
    wait(100_ms);
    goalClamp->unclamp();
    moveTo->forward({2_tile, 2_tile});
    turn->toward(270_deg);
    /*intake->load();
    moveTo->forward({0_tile, 2.5_tile});
    wait(100_ms);
    turn->toward(0_deg);
    moveTo->reverse({0_tile, 2_tile});
    ladybrown->extend();
    moveTo->forward({0_tile, 3_tile});
    wait(100_ms);
    moveTo->reverse({0_tile, 2_tile});*/
    



  } else if(id == ID24) {
    /*
          ___ _ _  _ _   ___ _   _ _ _
         |_  ) | |( | ) / __| |_(_) | |___
          / /|_  _|V V  \__ \ / / | | (_-<
         /___| |_|      |___/_\_\_|_|_/__/

    */
   setupRoutine(
    { -2.5_tile, 0_tile, 90_deg});
    intake->setSortOutColor(ColorSensor::Color::None);
    goalRushWhenReady();
    intake->index();
    moveTo->forward({-2_tile, 0_tile});
    moveTo->reverse({-2.5_tile, 0_tile});
    intake->intake();
    wait(1000_ms);
    turn->toward(155_deg);
    moveTo->forward({-2_tile, -2_tile});
    intake->index();
    wait(100_ms);
    turn->toward(270_deg);
    moveTo->reverse({-1_tile, -2_tile},
                    LateralProfile::Parameters{30_in_per_s, 30_in_per_s_sq});
    wait(100_ms);
    goalClamp->clamp();
    wait(100_ms);
    turn->toward(0_deg);
    intake->intake(); 
    moveTo->forward({-1_tile, -.75_tile});
    wait(100_ms);
    turn->toward(135_deg);
    intake->intake();
    moveTo->forward({-2_tile, -2_tile});
    turn->toward(270_deg);
    moveTo->forward({-2_tile, -2_tile});
    turn->toward(225_deg);
    intake->intake();
    moveTo->forward({-2.75_tile, -2.75_tile});
    moveTo->reverse({-2_tile, -2_tile});
    turn->toward(45_deg);
    moveTo->reverse({-2.75_tile, -2.75_tile});
    goalClamp->unclamp();
  }

  END_ROUTINE

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
  moveTo->forward({-1_tile, 1_tile});
  intake->intake();

  moveTo->forward({-1_tile, 2.4625_tile},
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

  END_ROUTINE

  START_ROUTINE("Positive Mid")
  setupRoutine(
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
  moveTo->forward({-1_tile, -1_tile});
  intake->intake();

  moveTo->forward({-1_tile, -2.4625_tile},
                  LateralProfile::Parameters{40_in_per_s});

  wait(1_s);
  moveTo->forward({-2_tile, -2_tile},
                  LateralProfile::Parameters{40_in_per_s});
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
  moveTo->forward({-1_tile, 0_tile},
                    LateralProfile::Parameters{40.5_in_per_s});

  END_ROUTINE

  START_ROUTINE("Do Nothing")
  setupRoutine({});
  END_ROUTINE
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
