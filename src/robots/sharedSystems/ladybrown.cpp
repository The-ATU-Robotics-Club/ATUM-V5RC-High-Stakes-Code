#include "ladybrown.hpp"

namespace atum {
Ladybrown::Ladybrown(std::unique_ptr<Motor> iLeft,
                     std::unique_ptr<Motor> iRight,
                     std::unique_ptr<Piston> iPiston,
                     std::unique_ptr<RotationSensor> iRotation,
                     std::unique_ptr<LineTracker> iLine,
                     const Parameters &iParams,
                     std::unique_ptr<AngularProfileFollower> iFollower,
                     const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    left{std::move(iLeft)},
    right{std::move(iRight)},
    piston{std::move(iPiston)},
    rotation{std::move(iRotation)},
    line{std::move(iLine)},
    params{iParams},
    follower{std::move(iFollower)},
    logger{loggerLevel} {
  left->setBrakeMode(pros::MotorBrake::brake);
  right->setBrakeMode(pros::MotorBrake::brake);
  left->resetPosition(params.absoluteStartingPosition);
  right->resetPosition(params.absoluteStartingPosition);
  rotation->resetDisplacement(params.absoluteStartingPosition);
  // Manually set state to avoid setting hold position.
  state = LadybrownState::Idle;
}

void Ladybrown::stop() {
  params.holdController.reset();
  state = LadybrownState::Idle;
  // During manual control, hold wherever you stop.
  if(!holdPosition.has_value()) {
    holdPosition = getPosition();
  }
}

void Ladybrown::extend() {
  state = LadybrownState::Extending;
  // During manual control, hold wherever you stop.
  holdPosition = {};
}

void Ladybrown::retract() {
  state = LadybrownState::Retracting;
  // During manual control, hold wherever you stop.
  holdPosition = {};
}

void Ladybrown::rest() {
  state = LadybrownState::Resting;
  holdPosition = params.statePositions[state];
}

void Ladybrown::load() {
  state = LadybrownState::Loading;
  holdPosition = params.statePositions[state];
}

void Ladybrown::prepare() {
  state = LadybrownState::Preparing;
  holdPosition = params.statePositions[state];
}

void Ladybrown::score() {
  // Don't override back up on the arm after scoring.
  if(state == LadybrownState::FinishScoring) {
    return;
  }
  state = LadybrownState::Scoring;
  holdPosition = params.statePositions[state];
}

LadybrownState Ladybrown::getClosestNamedPosition() const {
  // Don't give a measurement if you are still moving.
  if(state != LadybrownState::Idle) {
    return LadybrownState::Idle;
  }
  degree_t shortestDistance{std::numeric_limits<double>::max()};
  LadybrownState closestPosition{LadybrownState::Resting};
  for(const auto &[position, angle] : params.statePositions) {
    const degree_t distance{abs(angle - getPosition())};
    if(distance <= shortestDistance) {
      shortestDistance = distance;
      closestPosition = position;
    }
  }
  return closestPosition;
}

bool Ladybrown::hasRing() const {
  return getClosestNamedPosition() != LadybrownState::Resting &&
         line->triggered();
}

bool Ladybrown::readyToScore() {
  return getClosestNamedPosition() != LadybrownState::Loading && hasRing();
}

bool Ladybrown::mayConflictWithIntake() {
  return getClosestNamedPosition() == LadybrownState::Loading && hasRing();
}

void Ladybrown::finishScore() {
  state = LadybrownState::FinishScoring;
  holdPosition = params.statePositions[LadybrownState::Preparing];
}

void Ladybrown::moveTo(const degree_t target) {
  const LadybrownState startingState{state};
  follower->startProfile(getPosition(), target);
  while(!follower->isDone() && state == startingState) {
    const double followerOutput{
        follower->getOutput(getPosition(), getVelocity())};
    voltage = followerOutput;
    wait(5_ms);
  }
  stop();
}

degree_t Ladybrown::getPosition() const {
  if(rotation->check()) {
    // Just use reading from rotation sensor if available.
    return rotation->getDisplacement();
  }
  std::vector<degree_t> readings;
  if(left->check()) {
    readings.push_back(left->getPosition());
  }
  if(right->check()) {
    readings.push_back(right->getPosition());
  }
  return average(readings);
}

degrees_per_second_t Ladybrown::getVelocity() const {
  std::vector<degrees_per_second_t> readings;
  if(rotation->check()) {
    readings.push_back(rotation->getVelocity());
  }
  if(left->check()) {
    readings.push_back(left->getVelocity());
  }
  if(right->check()) {
    readings.push_back(right->getVelocity());
  }
  return average(readings);
}

void Ladybrown::handlePiston() {
  if(getPosition() >= params.flippingPosition) {
    piston->extend();
  } else {
    if(piston->isExtended()) {
      piston->retract();
      left->brake();
      right->brake();
      wait(params.pistonDelay);
    }
  }
}

double Ladybrown::getHoldOutput() {
  double holdOutput{0.0};
  if(state == LadybrownState::Idle) {
    const double holdError{
        getValueAs<degree_t>(holdPosition.value() - getPosition())};
    const double hold{params.holdController.getOutput(holdError)};
    holdOutput += hold;
  }
  holdOutput += params.kG * cos(getValueAs<radian_t>(getPosition()));
  return holdOutput;
}

TASK_DEFINITIONS_FOR(Ladybrown) {
  START_TASK("Ladybrown State Machine")
  while(true) {
    switch(state) {
      case LadybrownState::Idle: voltage = 0; break;
      case LadybrownState::Extending: voltage = params.manualVoltage; break;
      case LadybrownState::Retracting: voltage = -params.manualVoltage; break;
      case LadybrownState::Resting:
      case LadybrownState::Loading:
      case LadybrownState::Preparing:
        moveTo(params.statePositions[state]);
        break;
      case LadybrownState::Scoring:
        moveTo(params.statePositions[state]);
        finishScore();
        break;
      case LadybrownState::FinishScoring:
        moveTo(params.statePositions[LadybrownState::Preparing]);
        break;
      default: break;
    }
    wait();
  }
  END_TASK

  START_TASK("Ladybrown Control")
  while(true) {
    wait(5_ms); // At the top because of continue statement below.
    handlePiston();
    if(getPosition() <= params.noMovePosition && voltage <= 0) {
      left->brake();
      right->brake();
      continue;
    }
    double output{voltage + getHoldOutput()};
    double balance{0.0};
    if(left->check() && right->check()) {
      balance = params.balanceController.getOutput(
          getValueAs<degree_t>(left->getPosition()),
          getValueAs<degree_t>(right->getPosition()));
    }

    // If one is still working, reset the position of the others so everything
    // works if it comes back online.
    if(!left->check()) {
      left->resetPosition(getPosition());
    }
    if(!right->check()) {
      right->resetPosition(getPosition());
    }

    left->moveVoltage(output + balance);
    right->moveVoltage(output);
  }
  END_TASK
} // namespace atum
} // namespace atum