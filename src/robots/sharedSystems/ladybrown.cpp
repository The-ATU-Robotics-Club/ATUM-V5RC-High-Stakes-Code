#include "ladybrown.hpp"

namespace atum {
Ladybrown::Ladybrown(std::unique_ptr<Motor> iMotor,
                     std::unique_ptr<DistanceSensor> iDistance,
                     std::unique_ptr<RotationSensor> iRotation,
                     const Parameters &iParams,
                     std::unique_ptr<AngularProfileFollower> iFollower,
                     const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    motor{std::move(iMotor)},
    distance{std::move(iDistance)},
    rotation{std::move(iRotation)},
    params{iParams},
    follower{std::move(iFollower)},
    logger{loggerLevel} {
  // Reset devices.
  motor->setBrakeMode(pros::MotorBrake::brake);
  motor->resetPosition();
  rotation->resetDisplacement();
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

void Ladybrown::moveTo(const degree_t iTarget) {
  target = iTarget;
  nextState = {};
  state = LadybrownState::MovingTo;
}

bool Ladybrown::ringInCarriage() const {
  return state != LadybrownState::Resting &&
         distance->getDistance() <= params.loadRingDistance;
}

bool Ladybrown::ringInIndexer() const {
  return state == LadybrownState::Resting &&
         distance->getDistance() <= params.indexRingDistance;
}

bool Ladybrown::checkRingDetection() const {
  return distance->check();
}

degree_t Ladybrown::getPosition() const {
  if(rotation->check()) {
    return rotation->getDisplacement();
  }
  return motor->getPosition();
}

degrees_per_second_t Ladybrown::getVelocity() const {
  if(rotation->check()) {
    const degrees_per_second_t rotationV{rotation->getVelocity()};
    const degrees_per_second_t motorV{motor->getVelocity()};
    if(abs(rotationV) > abs(motorV)) {
      return rotationV;
    }
    return motorV;
  }
  return motor->getVelocity();
}

void Ladybrown::moveToControls() {
  // const LadybrownState startingState{state};
  // const degree_t startingTarget{target};
  // follower->startProfile(rotation->getDisplacement(), target);
  // while(!follower->isDone() && state == startingState &&
  //       target == startingTarget) {
  //   const double followerOutput{
  //       follower->getOutput(rotation->getDisplacement(), motor->getVelocity())};
  //   voltage = followerOutput;
  //   wait();
  // }
  // if(target != startingTarget) {
  //   moveToControls();
  // }
  stop();
}

double Ladybrown::getHoldOutput() {
  if(state == LadybrownState::Resting) {
    return 0.0;
  }
  double output{0.0};
  if(rotation->check()) {
    output = params.kG * cos(getValueAs<radian_t>(rotation->getPosition()));
  }
  if(state == LadybrownState::Idle || state == LadybrownState::Loading) {
    const double holdError{
        getValueAs<degree_t>(target - rotation->getDisplacement())};
    output += params.holdController.getOutput(holdError);
  }
  return output;
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
          target = rotation->getDisplacement();
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

    if((output < 0.0 && rotation->getDisplacement() < params.bounds.first) ||
       (output > 0.0 && rotation->getDisplacement() >= params.bounds.second)) {
      output = 0.0;
    }

    motor->moveVoltage(output);

    wait();
  }
  END_TASK
} // namespace atum
} // namespace atum