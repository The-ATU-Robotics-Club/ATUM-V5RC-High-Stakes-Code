#include "robotClone.hpp"
#include "robots/sharedSystems/intake.hpp"
#include "robots/sharedSystems/ladybrown.hpp"

namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.

// General Constants
static const second_t goalRushDelay{100_ms};
static const second_t goalClampDelay{100_ms};

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
void RobotClone::skills24() {
  setupRoutine({-2_tile, 2_tile, 90_deg});
}

ROUTINE_DEFINITIONS_FOR(RobotClone) {
  START_ROUTINE("Skills")
  if(id == ID15) {
    skills15();
  } else if(id == ID24) {
    skills24();
  }
  END_ROUTINE

  START_ROUTINE("Negative Side: N/M Rush")
  setupRoutine({-41.5_in, 30.5_in, 78_deg});
  goalRushL->extend();
  kaboomer->retract();
  moveToRush->forward(2_s, getInFrontOf(28_in), 12.0, false);
  goalRushL->retract();
  wait(goalRushDelay);
  moveToClose->reverse(3_s, {-2_tile, 1.2_tile}, 12.0, false);
  turn->toward(1_s, 90_deg + 30_deg);
  goalRushL->extend();
  moveToClose->forward(0.25_s, getInFrontOf(1_in), 12.0, false);
  turn->toward(1_s, 90_deg - 20_deg);
  goalRushL->retract();
  wait(goalRushDelay);
  moveToFar->reverse(2_s, {-0.75_tile, 1.375_tile}, 6.0);
  goalClamp->clamp();
  wait(goalClampDelay);

  intake->intake();
  moveToFar->forward(3_s, {-1.25_tile, 2.5_tile}, 5.0);
  wait(1_s);
  moveToClose->forward(3_s, {-1.5_tile, 1.5_tile, -45_deg}, 4.0);
  moveToClose->forward(2_s, {-2.25_tile, 2.25_tile}, 4.0);
  wait(.5_s);
  moveToClose->reverse(2_s, {-1.5_tile, 1.5_tile}, 4.0);
  moveToClose->forward(1.5_s, {-2.825_tile, 2.825_tile}, 4.5, false);
  moveToFar->reverse(1_s, {-2.2_tile, 2.2_tile}, 8.0, false);
  moveToFar->forward(1.5_s, {-2.825_tile, 2.825_tile}, 5, false);
  moveToFar->reverse(1_s, {-2.2_tile, 2.2_tile}, 8.0, false);
  intake->index();
  moveToFar->forward(1.5_s, {-2.825_tile, 2.825_tile}, 5, false);
  moveToFar->reverse(1_s, {-2_tile, 2_tile}, 8.0, false);

  moveToFar->forward(4_s, {-2_tile, 0_tile});
  intake->load();
  moveToClose->forward(2_s, {-3_tile, 0_tile}, 4);
  intake->finishLoading();
  moveToClose->reverse(2_s, getInFrontOf(-10_in));
  ladybrown->moveTo(190_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 2_s);
  ladybrown->moveTo(90_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Resting), 2_s);
  ladybrown->rest();

  moveToFar->forward(4_s, {-1.5_tile, -2_tile});
  turn->toward(2_s, {0_tile, 0_tile});
  END_ROUTINE

  START_ROUTINE("Do Nothing")
  setupRoutine({});
  intake->outtake();
  wait(3_s);
  intake->stop();
  END_ROUTINE

  START_ROUTINE("Positive Side: P Rush")
  setupRoutine({});
  moveToClose->reverse(2_s, {1_tile, 1.5_tile});
  clampWhenReady();
  moveToClose->reverse(2_s, {.5_tile, 1.75_tile});
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, 2.25_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveToFar->forward(2_s, {1_tile, 0_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: P Rush")
  setupRoutine({});
  moveToClose->reverse(2_s, {2_tile, .75_tile});
  clampWhenReady();
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  intake->load();
  moveToClose->forward(2_s, {2.5_tile, 0_tile, -90_deg});
  ladybrown->extend();
  ladybrown->retract();
  wait(100_ms);
  intake->intake();
  moveToFar->forward(2_s, {1_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 2.5_tile});
  END_ROUTINE

  START_ROUTINE("Positive Side: M Rush")
  setupRoutine({});
  intake->index();
  moveToClose->forward(2_s, {2.5_tile, -.5_tile});
  wait(100_ms);
  clampWhenReady();
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  wait(100_ms);
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(2_s, {2_tile, 1_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, 2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, 2.25_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveToFar->forward(2_s, {1_tile, 0_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: M Rush")
  setupRoutine({});
  clampWhenReady();
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  intake->stop();
  moveToClose->forward(2_s, {1_tile, -1_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, -2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  intake->stop();
  intake->load();
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  ladybrown->extend();
  wait(100_ms);
  ladybrown->retract();
  moveToFar->forward(2_s, {2.5_tile, 2.5_tile});
  END_ROUTINE

  START_ROUTINE("Positive Side: N Rush")
  setupRoutine({});
  intake->index();
  clampWhenReady();
  moveToClose->forward(2_s, {2.5_tile, -.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 1_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, 2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, 2.25_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveToFar->forward(2_s, {1_tile, 0_tile});

  END_ROUTINE

  START_ROUTINE("Negative Side: N Rush")
  setupRoutine({});
  clampWhenReady();
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(2_s, {1_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, -2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  intake->stop();
  intake->load();
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  ladybrown->extend();
  wait(100_ms);
  ladybrown->retract();
  moveToFar->forward(2_s, {2.5_tile, 2.5_tile});
  END_ROUTINE

  START_ROUTINE("Positive Side: P/N Rush")
  setupRoutine({-50.5_in, -27_in, 102_deg});

  rush(true, true);

  intake->intake();
  wait(100_ms);
  moveToClose->forward(5_s, {-1_tile, -1.5_tile});
  goalClamp->unclamp();
  moveToClose->forward(5_s, {-2.4_tile, -1_tile}, 6.0);
  intake->index();
  moveToFar->forward(5_s, {-2.5_tile, 0.5_tile});
  clampWhenReady();
  moveToClose->reverse(5_s, {-1.8_tile, -0.2_tile}, 6.0);
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(5_s, {-2_tile, 1_tile});
  wait(1_s);
  moveToFar->forward(5_s, {-0.75_tile, -2.25_tile}, 5);
  wait(1_s);
  moveToClose->forward(5_s, {-2.1_tile, -2.0_tile});
  wait(1_s);
  moveToClose->forward(5_s, {-3_tile, -3_tile}, 5);
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile}, 5);
  moveToClose->forward(5_s, {-3_tile, -3_tile}, 5);
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile}, 5);
  moveToClose->forward(5_s, {-3_tile, -3_tile}, 5);
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile}, 5);
  moveToClose->reverse(5_s, {-3_tile, -3_tile}, 5);
  goalClamp->toggleClamp();
  moveToFar->forward(5_s, {-1_tile, 3_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: P/N Rush")
  setupRoutine({});
  moveToClose->reverse(5_s, {-1_tile, 1.5_tile});
  goalRushL->extend();
  clampWhenReady();
  moveToClose->reverse(5_s, {-0.5_tile, 1.25_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(5_s, {-1.2_tile, 2.2_tile});
  moveToClose->forward(5_s, {-2_tile, 2_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  ladybrown->load();
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.75_tile, 2.75_tile});
  moveToFar->forward(5_s, {-2.8_tile, 0.2_tile});
  turn->toward(5_s, {-3_tile, 0_tile});
  ladybrown->extend();
  wait(500_ms);
  ladybrown->rest();
  wait(100_ms);
  moveToClose->forward(5_s, {-2.75_tile, -2.75_tile});

  END_ROUTINE

  START_ROUTINE("Positive Side: N/M Rush")
  setupRoutine({});
  moveToFar->reverse(5_s, {-1_tile, -1_tile});
  clampWhenReady();
  moveToClose->reverse(5_s, {-0.5_tile, -0.50_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  moveToFar->forward(5_s, {-2.5_tile, -1_tile});
  intake->index();
  moveToFar->forward(5_s, {-2.5_tile, 0.5_tile});
  clampWhenReady();
  moveToClose->reverse(5_s, {-1.8_tile, -0.2_tile});
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(5_s, {-2_tile, 1_tile});
  wait(1_s);
  moveToFar->forward(5_s, {-0.75_tile, -2.25_tile});
  moveToClose->forward(5_s, {-2.2_tile, -2.0_tile});
  wait(100_ms);
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->reverse(5_s, {-3_tile, -3_tile});
  goalClamp->toggleClamp();
  moveToFar->forward(5_s, {-1_tile, 3_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: N/M Rush")
  setupRoutine({});
  moveToClose->reverse(5_s, {-1_tile, 1.5_tile});
  goalRushL->extend();
  clampWhenReady();
  moveToClose->reverse(5_s, {-0.5_tile, 1.25_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(5_s, {-1.2_tile, 2.2_tile});
  moveToClose->forward(5_s, {-2_tile, 2_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  ladybrown->load();
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.75_tile, 2.75_tile});
  moveToFar->forward(5_s, {-2.8_tile, 0.2_tile});
  turn->toward(5_s, {-3_tile, 0_tile});
  ladybrown->extend();
  wait(500_ms);
  ladybrown->rest();
  wait(100_ms);
  moveToClose->forward(5_s, {-2.75_tile, -2.75_tile});

  END_ROUTINE

  // START_ROUTINE("Negative Side: N/M Rush")
  // setupRoutine({});
  // moveToFar->forward(2_s, {0.0_tile, 4_tile}, 12.0, false);
}

void RobotClone::rush(const bool right, const bool clampImmediately) {
  const int shouldFlipY{drive->getPose().y < 0_in ? -1.0 : 1.0};
  const int shouldFlipH{right ? -1.0 : 1.0};
  Piston *goalRush{right ? goalRushR.get() : goalRushL.get()};
  goalRush->extend();
  kaboomer->retract();
  moveToRush->forward(
      2_s,
      getInFrontOf(28_in + (id == ID15 ? extensionDistance : 0.0_in)),
      12.0,
      false);
  goalRush->retract();
  wait(goalRushDelay);
  moveToClose->reverse(3_s, {-2_tile, shouldFlipY * 1.2_tile}, 12.0, false);
  turn->toward(1_s, 90_deg + shouldFlipH * 30_deg);
  goalRush->extend();
  moveToClose->forward(0.25_s, getInFrontOf(1_in), 12.0, false);
  wait(goalRushDelay);
  turn->toward(1_s, 90_deg - shouldFlipH * 20_deg);
  goalRush->retract();
  wait(goalRushDelay);
  if(clampImmediately) {
    clampWhenReady();
  }
  moveToFar->reverse(2_s, {-0.75_tile, shouldFlipY * 1.375_tile}, 6.0);
  goalClamp->clamp();
  wait(goalClampDelay);
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
  goalRushL->retract();
  goalRushR->retract();

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
                        turn->interrupt();
                        moveToClose->interrupt();
                        pathFollower->interrupt();
                      },
                      timeout,
                      Scheduler::doNothing});
}

Pose RobotClone::getInFrontOf(const meter_t offset) const {
  const Pose current{drive->getPose()};
  const degree_t hAdj{90_deg - current.h};
  const Pose offset2d{offset * cos(hAdj), offset * sin(hAdj)};
  return current + offset2d;
}

Pose RobotClone::getPast(const Pose &target, const meter_t past) const {
  Pose current{drive->getPose()};
  if(GUI::Routines::selectedColor() == MatchColor::Blue) {
    current.flip();
  }
  const double hDiff{angle(current, target)};
  const Pose pastTarget{cos(hDiff) * past + target.x,
                        sin(hDiff) * past + target.y};
  return pastTarget;
}

void RobotClone::setSortToOpposite() {
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    intake->setSortOutColor(ColorSensor::Color::Blue);
  } else {
    intake->setSortOutColor(ColorSensor::Color::Red);
  }
}
} // namespace atum
