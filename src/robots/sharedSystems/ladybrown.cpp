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
  left->resetPosition();
  right->resetPosition();
  rotation->reset();
  stop();
}

void Ladybrown::stop() {
  state = LadybrownStates::Idle;
}

void Ladybrown::extend() {
  state = LadybrownStates::Extending;
}

void Ladybrown::retract() {
  state = LadybrownStates::Retracting;
}

void Ladybrown::load() {
  state = LadybrownStates::Loading;
}

void Ladybrown::score() {
  state = LadybrownStates::Scoring;
}

void Ladybrown::setVoltage(const double newVoltage) {
  std::scoped_lock lock{voltageMutex};
  voltage = newVoltage;
}

double Ladybrown::getVoltage() {
  std::scoped_lock lock{voltageMutex};
  return voltage;
}

void Ladybrown::moveTo(const degree_t target) {
  follower->startProfile(rotation->getDisplacement(), target);
  while(!follower->isDone()) {
    const double followerOutput{
        follower->getOutput(rotation->getPosition(), rotation->getVelocity())};
    setVoltage(followerOutput);
    wait(5_ms);
  }
  state = LadybrownStates::Idle;
}

TASK_DEFINITIONS_FOR(Ladybrown) {
  START_TASK("Ladybrown State Machine")
  while(true) {
    switch(state) {
      case LadybrownStates::Idle: setVoltage(0); break;
      case LadybrownStates::Extending: setVoltage(params.maxVoltage); break;
      case LadybrownStates::Retracting: setVoltage(-params.maxVoltage); break;
      case LadybrownStates::Scoring: moveTo(params.scoredPosition); break;
      case LadybrownStates::Loading: moveTo(params.loadingPosition); break;
      default: break;
    }
    wait();
  }
  END_TASK

  START_TASK("Ladybrown Control")
  while(true) {
    rotation->getVelocity();
    if(rotation->getDisplacement() >= params.flippingPosition) {
      piston->extend();
    } else {
      piston->retract();
    }
    if(rotation->getDisplacement() <= params.restPosition &&
       getVoltage() <= 0) {
      left->setBrakeMode(pros::MotorBrake::brake);
      right->setBrakeMode(pros::MotorBrake::brake);
    } else {
      left->setBrakeMode(pros::MotorBrake::hold);
      right->setBrakeMode(pros::MotorBrake::hold);
    }
    const double output{getVoltage()};
    const double balance{params.balanceController.getOutput(
        left->getPosition(), right->getPosition())};
    if(output) {
      left->moveVoltage(output + balance);
      right->moveVoltage(output);
    } else { // TODO: Add custom hold code.
      left->brake();
      right->brake();
    }
    wait(5_ms);
  }
  END_TASK
}
} // namespace atum