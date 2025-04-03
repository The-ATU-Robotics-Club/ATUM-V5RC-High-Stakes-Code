#include "ladybrown.hpp"

namespace atum {
Ladybrown::Ladybrown(std::unique_ptr<Motor> iMotor,
                     std::unique_ptr<DistanceSensor> iDistance,
                     const Parameters &iParams,
                     std::unique_ptr<AngularProfileFollower> iFollower,
                     const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    motor{std::move(iMotor)},
    distance{std::move(iDistance)},
    params{iParams},
    follower{std::move(iFollower)},
    logger{loggerLevel} {
  // Reset devices.
  motor->setBrakeMode(pros::MotorBrake::brake);
  motor->resetPosition();
  stop();

  logger.info("Ladybrown is constructed!");
}

void Ladybrown::stop() {
  if(state == LadybrownState::MovingTo) {
    state = nextState.value_or(LadybrownState::Idle);
    nextState = {};
  } else if(state == LadybrownState::Extending ||
            state == LadybrownState::Retracting) {
    state = LadybrownState::Settling;
  }
}

void Ladybrown::extend() {
  if(state != LadybrownState::MovingTo) {
    state = LadybrownState::Extending;
  }
}

void Ladybrown::retract() {
  if(state != LadybrownState::MovingTo) {
    state = LadybrownState::Retracting;
  }
}

void Ladybrown::rest() {
  if(ringInCarriage()) {
    return;
  }
  if(state != LadybrownState::Resting) {
    nextState = LadybrownState::Resting;
    state = LadybrownState::MovingTo;
  }
  target = 0_deg;
}

void Ladybrown::load() {
  if(ringInCarriage()) {
    return;
  }
  if(state != LadybrownState::Loading) {
    nextState = LadybrownState::Loading;
    state = LadybrownState::MovingTo;
  }
  target = params.loadingPosition;
}

bool Ladybrown::ringInCarriage() const {
  return state != LadybrownState::Resting &&
         distance->getDistance() <= params.generalRingDistance;
}

bool Ladybrown::ringInIndexer() const {
  return state == LadybrownState::Resting &&
         distance->getDistance() <= params.restingRingDistance;
}

bool Ladybrown::checkRingDetection() const {
  return distance->check();
}

void Ladybrown::moveToControls() {
  const LadybrownState startingState{state};
  follower->startProfile(motor->getPosition(), target);
  while(!follower->isDone() && state == startingState) {
    const double followerOutput{
        follower->getOutput(motor->getPosition(), motor->getVelocity())};
    voltage = followerOutput;
    wait();
  }
  stop();
}

double Ladybrown::getHoldOutput() {
  if(state == LadybrownState::Idle || state == LadybrownState::Loading) {
    const double holdError{getValueAs<degree_t>(target - motor->getPosition())};
    return params.holdController.getOutput(holdError);
  }
  return 0.0;
}

TASK_DEFINITIONS_FOR(Ladybrown) {
  START_TASK("Ladybrown State Machine")
  while(true) {
    switch(state) {
      case LadybrownState::Resting:
      case LadybrownState::Idle:
      case LadybrownState::Loading: voltage = 0.0; break;
      case LadybrownState::Settling:
        voltage = 0.0;
        // During manual control, hold wherever you stop.
        if(abs(motor->getVelocity()) <= params.stillRPM) {
          target = motor->getPosition();
          state = LadybrownState::Idle;
        }
        break;
      case LadybrownState::Extending: voltage = params.manualVoltage; break;
      case LadybrownState::Retracting: voltage = -params.manualVoltage; break;
      default: moveToControls(); break;
    }
    wait();
  }
  END_TASK

  START_TASK("Ladybrown Control")
  while(true) {
    const double slewedVoltage{params.manualSlew.slew(voltage)};
    const bool macroMovement{state == LadybrownState::MovingTo};
    double output{macroMovement ? voltage : slewedVoltage};
    output += getHoldOutput();

    motor->moveVoltage(output);

    wait();
  }
  END_TASK
} // namespace atum
} // namespace atum