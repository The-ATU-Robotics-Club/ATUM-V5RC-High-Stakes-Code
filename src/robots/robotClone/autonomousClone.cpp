#include "robotClone.hpp"


namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.

// General Constants
static const LateralProfile::Parameters goalClampProfile{35_in_per_s,
                                                         70_in_per_s_sq};
static const LateralProfile::Parameters cornerMotionProfile{30_in_per_s,
                                                            90_in_per_s_sq};
LateralProfile::Parameters doubleStackProfile{40_in_per_s, 50_in_per_s_sq};
static const inches_per_second_t singleRingSpeed{60_in_per_s};
static const tile_t pushDoubleStackY{2.47_tile};
static const second_t goalRushDelay{100_ms};
static const second_t goalClampDelay{100_ms};
static const second_t singleRingDelay{0.825_s};
static const second_t doubleRingDelay{1.125_s};

// DATA:
// ODOMS DRIVE
// 12.2878 9.39187
// 12.334 9.40656
// 12.3254 9.4

/*
  _ ___ _ _   ___ _   _ _ _
 / | __( | ) / __| |_(_) | |___
 | |__ \V V  \__ \ / / | | (_-<
 |_|___/     |___/_\_\_|_|_/__/

*/
void RobotClone::skills15() {}

/*
      ___ _ _  _ _   ___ _   _ _ _
     |_  ) | |( | ) / __| |_(_) | |___
      / /|_  _|V V  \__ \ / / | | (_-<
     /___| |_|      |___/_\_\_|_|_/__/

*/
void RobotClone::skills24() {}

ROUTINE_DEFINITIONS_FOR(RobotClone) {
  START_ROUTINE("Skills")
  if(id == ID15) {
    skills15();
  } else if(id == ID24) {
    skills24();
  }
  END_ROUTINE

  START_ROUTINE("Do Nothing")
  setupRoutine({});
  intake->outtake();
  wait(3_s);
  intake->stop();
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
  goalRush->release();

  drive->setBrakeMode(pros::MotorBrake::brake);
}

void RobotClone::clampWhenReady(const second_t timeout) {
  scheduler.schedule({"Clamp When Ready",
                      [=]() { return goalClamp->hasGoal(); },
                      [=]() {
                        if(goalClamp->isClamped()) {
                          return;
                        }
                        goalClamp->clamp();
                        wait(goalClampDelay);
                        turn->interrupt();
                        moveTo->interrupt();
                        pathFollower->interrupt();
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
                        goalRush->grab();
                        wait(goalRushDelay);
                        turn->interrupt();
                        moveTo->interrupt();
                        pathFollower->interrupt();
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
