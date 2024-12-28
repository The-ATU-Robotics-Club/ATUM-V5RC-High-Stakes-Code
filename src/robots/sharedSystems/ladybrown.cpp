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
  left->resetPosition();
  right->resetPosition();
  rotation->reset();
  // Manually set state to avoid setting hold position.
  state = LadybrownState::Idle;
}

void Ladybrown::stop() {
  // During manual control, hold wherever you stop.
  if(!holdPosition.has_value()) {
    holdPosition = getPosition();
  }
  params.holdController.reset();
  state = LadybrownState::Idle;
}

void Ladybrown::extend() {
  // During manual control, hold wherever you stop.
  state = LadybrownState::Extending;
  holdPosition = {};
}

void Ladybrown::retract() {
  // During manual control, hold wherever you stop.
  state = LadybrownState::Retracting;
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
    holdPosition = params.statePositions[LadybrownState::Preparing];
    return;
  }
  state = LadybrownState::Scoring;
  holdPosition = params.statePositions[state];
}

bool Ladybrown::mayConflictWithIntake() {
  return getClosestNamedPosition() == LadybrownState::Loading && hasRing();
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

void Ladybrown::moveTo(const degree_t target) {
  const LadybrownState startingState{state};
  follower->startProfile(getPosition(), target);
  while(!follower->isDone() && state == startingState) {
    const double followerOutput{
        follower->getOutput(getPosition(), getVelocity())};
    setVoltage(followerOutput);
    wait(5_ms);
  }
  stop();
}

degree_t Ladybrown::getPosition() const {
  return getPosition();
}

degrees_per_second_t Ladybrown::getVelocity() const {
  const degrees_per_second_t leftMotorVelocity{0.2 * left->getVelocity()};
  const degrees_per_second_t rightMotorVelocity{0.2 * right->getVelocity()};
  const degrees_per_second_t rotationVelocity{rotation->getVelocity()};
  return (leftMotorVelocity + rightMotorVelocity + rotationVelocity) / 3.0;
}

void Ladybrown::setVoltage(const double newVoltage) {
  std::scoped_lock lock{voltageMutex};
  voltage = newVoltage;
}

double Ladybrown::getVoltage() {
  std::scoped_lock lock{voltageMutex};
  return voltage;
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
  const degree_t absolutePosition{params.absoluteStartingPosition +
                                  getPosition()};
  holdOutput += params.kG * cos(getValueAs<radian_t>(absolutePosition));
  return holdOutput;
}

TASK_DEFINITIONS_FOR(Ladybrown) {
  START_TASK("Ladybrown State Machine")
  while(true) {
    switch(state) {
      case LadybrownState::Idle: setVoltage(0); break;
      case LadybrownState::Extending: setVoltage(params.manualVoltage); break;
      case LadybrownState::Retracting: setVoltage(-params.manualVoltage); break;
      case LadybrownState::Resting:
      case LadybrownState::Loading:
      case LadybrownState::Preparing:
        moveTo(params.statePositions[state]);
        break;
      case LadybrownState::Scoring:
        moveTo(params.statePositions[state]);
        state = LadybrownState::FinishScoring;
        break;
      case LadybrownState::FinishScoring:
        holdPosition = params.statePositions[LadybrownState::Preparing];
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
    if(getPosition() <= params.noMovePosition && getVoltage() <= 0) {
      left->brake();
      right->brake();
      continue;
    }
    double output{getVoltage() + getHoldOutput()};
    const double balance{params.balanceController.getOutput(
        left->getPosition(), right->getPosition())};
    left->moveVoltage(output + balance);
    right->moveVoltage(output);
  }
  END_TASK
} // namespace atum
} // namespace atum