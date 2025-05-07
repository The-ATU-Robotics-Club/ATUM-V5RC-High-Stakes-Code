#include "robotClone.hpp"
#include "robots/sharedSystems/intake.hpp"
#include "robots/sharedSystems/ladybrown.hpp"

namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.

// General Constants
static const Pose noRushStart{-2.5_tile, -1_tile, 0_deg};
static const second_t goalRushDelay{100_ms};
static const second_t goalClampDelay{100_ms};

/*
  _ ___ _ _   ___ _   _ _ _
 / | __( | ) / __| |_(_) | |___
 | |__ \V V  \__ \ / / | | (_-<
 |_|___/     |___/_\_\_|_|_/__/

*/
void RobotClone::skills15() {
  setupRoutine({-2.5_tile + 7.5_in, 1_tile, 90_deg});
  intake->setSortOutColor(ColorSensor::Color::Blue);

  intake->index();
  moveToClose->forward(5_s, {-1_tile, 1_tile}, 8);
  clampWhenReady();
  moveToClose->reverse(3_s, {-1_tile, 2.2_tile}, 4);
  goalClamp->clamp();
  wait(goalClampDelay);
  intake->intake();

  moveToClose->forward(5_s, {0_tile, 2_tile}, 8);
  wait(200_ms);
  moveToClose->reverse(5_s, {-1_tile, 2_tile}, 8);
  moveToClose->forward(5_s, {-2_tile, 2_tile}, 8);

  moveToClose->forward(1.5_s, {-3_tile, 3_tile}, 4);
  moveToClose->reverse(1.5_s, getInFrontOf(-4.5_in), 6.0, false);
  wait(100_ms);
  moveToClose->reverse(3_s, {-2.75_tile, 2.75_tile}, 6);
  goalClamp->unclamp();
  moveToClose->forward(2_s, {-2.25_tile, 2.25_tile}, 6);

  intake->load();

  moveToClose->forward(4_s, {0_tile, 2.45_tile}, 8);
    
wait(100_ms);
moveToClose->reverse(3_s, {0_tile, 2.2_tile}, 6);
ladybrown->load();
moveToClose->forward(3_s, {0_tile, 2.75_tile}, 4);
ladybrown->moveTo(170_deg);
waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
ladybrown->moveTo(60_deg);
waitUntil(ladybrown->checkStateIs(LadybrownState::Resting), 1_s);
ladybrown->rest();
moveToClose->reverse(2_s, {0_tile, 1.75_tile});
ladybrown->rest();
clampWhenReady();
moveToClose->reverse(5_s, {1.25_tile, 0.75_tile}, 8);
goalClamp->clamp();
wait(goalClampDelay);
intake->intake();
moveToClose->forward(5_s, {1_tile, 2_tile}, 10);
wait(200_ms);
moveToClose->forward(5_s, {2_tile, 1_tile}, 10);
wait(200_ms);
moveToClose->forward(5_s, {2_tile, 2_tile}, 10);
wait(200_ms);
intake->load();
moveToClose->forward(2_s, {2.8_tile, 2.8_tile}, 8);
moveToClose->reverse(2_s, {2.4_tile, 2.4_tile});
wait(200_ms);
moveToClose->reverse(2_s, {2.9_tile, 2.9_tile}, 8);
goalClamp->unclamp();
wait(goalClampDelay);
ladybrown->moveTo(60_deg);
moveToClose->forward(5_s, {2.5_tile, 2_tile});
intake->index();
moveToFar->forward(5_s, {2.5_tile, -0.5_tile}, 9);
wait(200_ms);
clampWhenReady();
moveToClose->reverse(5_s, {1.8_tile, 0.2_tile}, 6);
goalClamp->clamp();
intake->intake();
  moveToClose->forward(1_s, {3_tile, 0_tile}, 4);
  if(intake->getColor() != intake->getSortOutColor()) {
    intake->finishLoading();
  }
  moveToClose->reverse(2_s, getInFrontOf(-9.5_in));
  ladybrown->moveTo(185_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  moveToClose->reverse(2_s, getInFrontOf(-4_in));
  ladybrown->moveTo(90_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  ladybrown->rest();
  moveToFar->reverse(5_s, {1_tile, 1_tile}, 10);
  ladybrown->moveTo(135_deg);
  moveToClose->reverse(5_s, {0_tile, 0_tile}, 6);
  moveToClose->forward(2_s, {1_tile, 1_tile});
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
  
  intake->index();
  moveToClose->forward(2_s, {-2_tile, 0_tile});
  wait(300_ms);
  intake->load();
 moveToClose->forward(1_s, {-3_tile, 0_tile}, 4);
  if(intake->getColor() != intake->getSortOutColor()) {
    intake->finishLoading();
  }
  moveToClose->reverse(2_s, getInFrontOf(-9.5_in));
  ladybrown->moveTo(225_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  moveToClose->reverse(2_s, getInFrontOf(-4_in));
  ladybrown->moveTo(90_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  ladybrown->rest();
 
  intake->index();
  moveToClose->forward(2_s, {-1_tile, -1_tile}, 4);
  wait(100_ms);
  clampWhenReady();
  moveToClose->reverse(2_s, {-1_tile, -2.25_tile}, 4);
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);

  moveToClose->forward(2_s, {.25_tile, -2_tile}, 6);
  wait(100_ms);
  moveToClose->reverse(2_s, {-.75_tile, -2_tile}, 4);

  moveToFar->forward(2_s, {-2_tile, -2_tile}, 6);
  wait(100_ms);
  
  moveToClose->forward(2_s, {-2.75_tile, -2.75_tile}, 6);
  wait(100_ms);
  moveToClose->reverse(2_s, {-2_tile, -2_tile}, 6);
  moveToClose->reverse(2_s, {-2.5_tile, -2.5_tile}, 6);
  goalClamp->unclamp();

  intake->index();
  moveToFar->forward(2_s, {0.25_tile, -2.6_tile}, 6);
  wait(100_ms);
  intake->load();
  moveToFar->forward(2_s, {0_tile, -2_tile}, 6);
  wait(200_ms);
  moveToClose->reverse(2_s, {0_tile, -2_tile}, 6);
  moveToClose->forward(2_s, {0_tile, -3_tile});
  wait(100_ms);
  intake->stop();
  ladybrown->moveTo(150_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  ladybrown->moveTo(90_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Resting), 1_s);
  ladybrown->rest();
  wait(100_ms);

  moveToClose->reverse(2_s, {0_tile, -1.75_tile}, 4);
  intake->index();
  moveToFar->forward(2_s, {1.2_tile, -2_tile}, 4);
  clampWhenReady();
  moveToClose->reverse(2_s, {1_tile, -.75_tile}, 4);
  goalClamp->clamp();
  wait(goalClampDelay);
  intake->intake();
  
  intake->setSortOutColor(ColorSensor::Color::Blue);

  moveToClose->forward(2_s, {2.25_tile, -1_tile}, 4);
  wait(100_ms);
  intake->intake();

  moveToClose->forward(2_s, {2_tile, -2_tile}, 4);
  wait(1_s); 
  intake->stop();
  
  moveToClose->reverse(2_s, {1.5_tile, -1.5_tile}, 4);
  intake->index();
  
  moveToClose->forward(2_s, {2.85_tile, -2.85_tile}, 8);
  wait(300_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile},6);
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, -2.75_tile},6);
  wait(100_ms);
  goalClamp->unclamp();

  moveToClose->forward(2_s, {2_tile, -2_tile},6);
  clampWhenReady();
  moveToFar->reverse(2_s, {2_tile, 0_tile}, 6);
  goalClamp->clamp();
  wait(goalClampDelay);
  intake->intake();
  wait(500_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile},6);
  intake->stop();
  goalClamp->unclamp();

  moveToClose->reverse(2_s, {2_tile, -2_tile},6);
  ladybrown->moveTo(135_deg);
  moveToClose->reverse(2_s, {0_tile, 0_tile}, 6);
}

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

  START_ROUTINE("Negative Side: N & M Rush")
  rushSetup(true, true);
  rush(true, false);
  collectNegative();
  allianceStake();
  moveToClose->forward(4_s, {-1_tile, 0_tile}, 6.0);
  END_ROUTINE

  START_ROUTINE("Positive Side: N & M Rush")
  rushSetup(false, true);
  rush(true, true);
  intake->intake();
  wait(2_s);
  goalClamp->unclamp();
  collectMiddle(false, true, true, false);
  endOfPositive({-1.8_tile, -1.8_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: N & P Rush")
  rushSetup(true, true);
  rush(true, false);
  collectNegative();
  allianceStake();
  moveToClose->forward(4_s, {-1_tile, 0_tile}, 6.0);
  END_ROUTINE

  START_ROUTINE("Positive Side: N & P Rush")
  rushSetup(false, false);
  rush(false, true);
  intake->intake();
  wait(2_s);
  goalClamp->unclamp();
  collectMiddle(false, true, true, false);
  endOfPositive({-1.8_tile, -1.8_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: M & P Rush")
  rushSetup(true, false);
  rush(false, true);
  collectNegative();
  allianceStake();
  moveToClose->forward(4_s, {-1_tile, 0_tile}, 6.0);
  END_ROUTINE

  START_ROUTINE("Positive Side: M & P Rush")
  rushSetup(false, false);
  rush(false, true);
  intake->intake();
  wait(2_s);
  goalClamp->unclamp();
  collectMiddle(false, true, true, false);
  endOfPositive({-1.8_tile, -1.8_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: N Only Rush")
  rushSetup(true, true);
  rush(true, false);
  collectNegative();
  allianceStake();
  moveToClose->forward(4_s, {-1_tile, 0_tile}, 6.0);
  END_ROUTINE

  START_ROUTINE("Positive Side: N Only Rush")
  setupRoutine(noRushStart);
  if(id == ID15) {
    intake->outtake();
    wait(0.25_s);
  }
  collectMiddle(false, false, false, true);
  endOfPositive({-1.8_tile, -1.8_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: M Only Rush")
  rushSetup(true, false);
  rush(false, true);
  collectNegative();
  allianceStake();
  moveToClose->forward(4_s, {-1_tile, 0_tile}, 6.0);
  END_ROUTINE

  START_ROUTINE("Positive Side: M Only Rush")
  setupRoutine(noRushStart);
  if(id == ID15) {
    intake->outtake();
    wait(0.25_s);
  }
  collectMiddle(false, false, false, true);
  endOfPositive({-1.8_tile, -1.8_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: P Only Rush")
  Pose negativeStart{noRushStart};
  negativeStart.y *= -1;
  negativeStart.h += 180_deg;
  setupRoutine(negativeStart);
  if(id == ID15) {
    intake->outtake();
    wait(0.25_s);
  }
  collectMiddle(true, false, true, true);
  collectNegative();
  allianceStake();
  intake->intake();
  moveToClose->reverse(2_s, {-2.4_tile, -2.4_tile});
  goalClamp->unclamp();
  moveToClose->forward(3_s, getInFrontOf(1.25_tile), 12.0, false);
  END_ROUTINE

  START_ROUTINE("Positive Side: P Only Rush")
  rushSetup(false, false);
  rush(false, false);
  endOfPositive({-1_tile, 0_tile});
  END_ROUTINE
}

void RobotClone::endOfPositive(const Pose &endPosition) {
  intake->intake();
  moveToFar->forward(5_s, getPast({-1_tile, -2_tile}, 1.3_ft), 5.0);
  moveToClose->forward(3_s, getPast({-2_tile, -2_tile}, 1.125_ft), 5.0);
  moveToClose->reverse(1_s, {-2_tile, -2_tile}, 12.0, false);
  collectCorner(false, 4, false);
  moveToClose->reverse(5_s, {-2_tile, -2_tile}, 12.0, false);
  moveToClose->reverse(2_s, {-2.5_tile, -2.5_tile});
  goalClamp->unclamp();
  moveToClose->forward(5_s, {-2_tile, -2_tile}, 12.0, false);
  moveToClose->forward(5_s, endPosition, 6.0);
}

void RobotClone::collectNegative() {
  intake->intake();
  moveToFar->forward(4_s, getPast({-1_tile, 2_tile}, 1.3_ft), 5.0);
  moveToClose->forward(3_s, getPast({-2_tile, 2_tile}, 1.125_ft), 5.0);
  moveToClose->reverse(1_s, {-2_tile, 2_tile}, 12.0, false);
  collectCorner(true, 3, true);
  moveToFar->reverse(1_s, {-2_tile, 2_tile}, 8.0, false);
}

void RobotClone::allianceStake() {
  moveToFar->forward(5_s, {-52_in, 0_tile});
  intake->load();
  moveToClose->forward(1_s, {-3_tile, 0_tile}, 4);
  if(intake->getColor() != intake->getSortOutColor()) {
    intake->finishLoading();
  }
  moveToClose->reverse(2_s, getInFrontOf(-9.5_in));
  ladybrown->moveTo(185_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  moveToClose->reverse(2_s, getInFrontOf(-4_in));
  ladybrown->moveTo(90_deg);
  waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 1_s);
  ladybrown->rest();
}

void RobotClone::collectMiddle(const bool negative,
                               const bool goToStart,
                               const bool otherPreload,
                               const bool startPreload) {
  const int shouldFlipY{negative ? -1.0 : 1.0};
  if(goToStart) {
    intake->intake();
    moveToClose->forward(5_s, {-2.4_tile, shouldFlipY * -1_tile}, 6.0);
  }
  intake->index();
  moveToFar->forward(5_s, {-2.5_tile, shouldFlipY * 0.5_tile}, 9.0);
  clampWhenReady();
  moveToClose->reverse(5_s, {-1.75_tile, shouldFlipY * -0.25_tile}, 6.0);
  goalClamp->clamp();
  wait(goalClampDelay);
  intake->intake();
  if(otherPreload) {
    moveToClose->forward(5_s, {-2_tile, shouldFlipY * 1.125_tile});
  }
  if(startPreload) {
    moveToFar->forward(5_s, {-2_tile, shouldFlipY * -1.125_tile});
  }
}

void RobotClone::rush(const bool left, const bool clampImmediately) {
  const int shouldFlipY{drive->getPose().y < 0_in ? -1.0 : 1.0};
  const int shouldFlipH{left ? 1.0 : -1.0};
  const bool middle{(left && shouldFlipY == -1.0) ||
                    (!left && shouldFlipY == 1.0)};
  Piston *goalRush{left ? goalRushL.get() : goalRushR.get()};
  scheduler.schedule({"Lower Goal Rush",
                      Scheduler::neverMet,
                      [=]() {
                        wait(150_ms);
                        intake->index();
                        wait(50_ms);
                        goalRush->extend();
                      },
                      100_ms});
  if(id == ID15) {
    intake->outtake();
  }
  kaboomer->retract();
  moveToRush->forward(
      2_s,
      getInFrontOf(id == ID15 ? 28_in + extensionDistance : 28_in),
      12.0,
      false);
  goalRush->retract();
  wait(goalRushDelay);
  if(middle) {
    moveToClose->reverse(3_s, {-1.75_tile, shouldFlipY * 0.5_tile}, 12.0, false);
  } else {
    moveToClose->reverse(3_s, {-1.75_tile, shouldFlipY * 1.5_tile}, 12.0, false);
  }
  turn->toward(1_s, 90_deg + shouldFlipH * 30_deg);
  goalRush->extend();
  moveToClose->forward(0.25_s, getInFrontOf(2_in), 12.0, false);
  wait(goalRushDelay);
  turn->toward(1_s, 90_deg - shouldFlipH * 30_deg);
  goalRush->retract();
  wait(goalRushDelay);
  if(clampImmediately) {
    clampWhenReady();
  }
  if(middle) {
    moveToFar->reverse(2_s, {-0.75_tile, shouldFlipY * 0.625_tile}, 6.0);
  } else {
    moveToFar->reverse(2_s, {-0.75_tile, shouldFlipY * 1.375_tile}, 6.0);
  }
  goalClamp->clamp();
  if(middle) {
    moveToClose->forward(3_s, {-1_tile, shouldFlipY * 1_tile});
  }
  wait(goalClampDelay);
}

void RobotClone::collectCorner(const bool negative,
                               const int pushes,
                               const bool shouldIndexLastRing) {
  const int shouldFlipY{negative ? 1 : -1};
  intake->intake();
  moveToClose->forward(1.5_s, {-3_tile, shouldFlipY * 3_tile}, 4);
  moveToClose->reverse(1.5_s, getInFrontOf(-4.5_in), 6.0, false);
  for(int i{0}; i < pushes - 2; i++) {
    moveToClose->forward(1.5_s, {-3_tile, shouldFlipY * 3_tile}, 4, false);
    moveToClose->reverse(1.5_s, getInFrontOf(-4.5_in), 6.0, false);
  }
  if(shouldIndexLastRing) {
    intake->index();
  }
  moveToClose->forward(1.5_s, {-3_tile, shouldFlipY * 3_tile}, 4, false);
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
  Pose target{current + offset2d};
  target.h = current.h;
  return target;
}

Pose RobotClone::getPast(const Pose &target, const meter_t past) const {
  Pose current{drive->getPose()};
  if(GUI::Routines::selectedColor() == MatchColor::Blue) {
    current.flip();
  }
  const double hDiff{getValueAs<radian_t>(90_deg - angle(current, target))};
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

void RobotClone::rushSetup(const bool negative, const bool rushingLeft) {
  Pose basePose{-41.5_in, 30.5_in, 90_deg};
  const degree_t angleOffset{11_deg};
  if(rushingLeft) {
    if(!negative) {
      basePose.y -= 1_tile;
      basePose.y *= -1;
      basePose.y += 1_tile;
      basePose.y *= -1;
    }
    basePose.h -= angleOffset;
  } else {
    if(negative) {
      basePose.y -= 1_tile;
      basePose.y *= -1;
      basePose.y += 1_tile;
    } else {
      basePose.y *= -1;
    }
    basePose.h += angleOffset;
  }
  setupRoutine(basePose);
  if(id == ID15) {
    wait();
    drive->setPose(getInFrontOf(-extensionDistance));
  }
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
} // namespace atum
