#include "atum/devices/colorSensor.hpp"
#include "ladybrown.hpp"


namespace atum {
Ladybrown::Ladybrown(std::unique_ptr<Motor> iMotor,
                     std::unique_ptr<DistanceSensor> iDistance,
                     std::unique_ptr<RotationSensor> iRotation,
                     const Parameters &iParams,
                     const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    motor{std::move(iMotor)},
    distance{std::move(iDistance)},
    rotation{std::move(iRotation)},
    params{iParams},
    logger{loggerLevel} {
  // Reset devices.
  motor->setBrakeMode(pros::MotorBrake::brake);
  motor->resetPosition();
  rotation->resetDisplacement();
  state = LadybrownState::Resting;
  logger.info("Ladybrown is constructed!");
}

void Ladybrown::setIntake(Intake *iIntake) {
  intake = iIntake;
}

void Ladybrown::stop() {
  nextState = {};
  if(state == LadybrownState::MovingTo) {
    state = nextState.value_or(LadybrownState::Idle);
  } else if(state == LadybrownState::Extending ||
            state == LadybrownState::Retracting) {
    state = LadybrownState::Settling;
  }
}

void Ladybrown::extend() {
  if(state != LadybrownState::MovingTo && state != LadybrownState::Packing) {
    state = LadybrownState::Extending;
  }
}

void Ladybrown::retract() {
  if(state == LadybrownState::MovingTo) {
    return;
  }
  if(getPosition() <= params.bounds.first) {
    rest();
  } else {
    state = LadybrownState::Retracting;
  }
}

void Ladybrown::rest() {
  if(ringInCarriage()) {
    return;
  }
  if(state != LadybrownState::Resting) {
    params.acceptable.reset();
    nextState = LadybrownState::Resting;
    state = LadybrownState::MovingTo;
  }
  target = 0_deg;
}

void Ladybrown::load() {
  if(ringInCarriage() || state == LadybrownState::Loading) {
    return;
  }
  moveTo(params.loadingPosition, LadybrownState::Loading);
}

void Ladybrown::pack() {
  params.acceptable.reset();
  nextState = state;
  state = LadybrownState::Packing;
}

void Ladybrown::prepare() {
  if(intake->getState() != IntakeState::Pressed) {
    return;
  }
  if(target != params.preparedPosition) {
    params.acceptable.reset();
  }
  target = params.preparedPosition;
  nextState = LadybrownState::Idle;
  state = LadybrownState::Preparing;
}

void Ladybrown::moveTo(const degree_t iTarget,
                       const std::optional<LadybrownState> &iNextState) {
  if(target != iTarget) {
    params.acceptable.reset();
  }
  target = iTarget;
  nextState = iNextState;
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

void Ladybrown::moveToControls() {
  if(state != LadybrownState::Idle && state != LadybrownState::MovingTo) {
    return;
  }
  double output{0.0};
  if(rotation->check()) {
    output = params.kG * cos(getValueAs<radian_t>(rotation->getPosition()));
  }
  const degree_t error{target - rotation->getDisplacement()};
  output += params.holdController.getOutput(getValueAs<degree_t>(error));
  voltage = output;
  if(params.acceptable.canAccept(error)) {
    state = nextState.value_or(LadybrownState::Idle);
  }
}

TASK_DEFINITIONS_FOR(Ladybrown) {
  START_TASK("Ladybrown State Machine")
  while(true) {
    switch(state) {
      case LadybrownState::Resting: voltage = 0.0; break;
      case LadybrownState::Settling:
        voltage = 0.0;
        // During manual control, hold wherever you stop.
        if(abs(motor->getVelocity()) <= params.stillRPM) {
          target = rotation->getDisplacement();
          state = LadybrownState::Idle;
        }
        break;
      case LadybrownState::Extending:
        intake->finishLoading();
        if(intake->getState() == IntakeState::Pressed ||
           intake->getState() == IntakeState::UnpressLoading) {
          voltage = 0.0;
          break;
        } else {
          voltage = params.manualVoltage;
        }
        break;
      case LadybrownState::Packing:
        voltage = -12.0;
        wait(params.packingTime);
        voltage = 0.0;
        state = nextState.value_or(LadybrownState::Idle);
        break;
      case LadybrownState::Retracting: voltage = -params.manualVoltage; break;
      case LadybrownState::Preparing: intake->finishLoading(); [[fallthrough]];
      default:
        intake->finishLoading();
        moveToControls();
        break;
    }
    wait();
  }
  END_TASK

  START_TASK("Ladybrown Control")
  while(true) {
    double output{params.manualSlew.slew(voltage)};

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