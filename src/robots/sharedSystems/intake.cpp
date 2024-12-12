#include "intake.hpp"

namespace atum {
Intake::Intake(std::unique_ptr<Motor> iMtr,
               std::unique_ptr<ColorSensor> iColorSensor,
               const Parameters &iParams,
               const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    mtr{std::move(iMtr)},
    colorSensor{std::move(iColorSensor)},
    logger{loggerLevel},
    params{iParams} {
  logger.info("Intake is constructed!");
}

void Intake::intake() {
  if(state == IntakeState::Jammed || state == IntakeState::Sorting) {
    return;
  }
  forceIntake(IntakeState::Intaking);
}

void Intake::index() {
  if(state == IntakeState::Jammed || state == IntakeState::Sorting) {
    return;
  }
  forceIntake(IntakeState::Indexing);
}

void Intake::outtake() {
  state = IntakeState::Outtaking;
}

void Intake::stop() {
  state = IntakeState::Idle;
}

void Intake::setAntiJam(const bool iAntiJamEnabled) {
  antiJamEnabled = iAntiJamEnabled;
}

void Intake::intaking() {
  if(sortOutColor != ColorSensor::Color::None &&
     colorSensor->getColor() == sortOutColor) {
    logger.debug("Switching to sorting!");
    state = IntakeState::Sorting;
  }
  if(state == IntakeState::Indexing) {
    if(colorSensor->getColor() != ColorSensor::Color::None) {
      params.timerUntilJamChecks.resetAlarm();
      mtr->brake();
      return;
    }
  }
  mtr->moveVoltage(12);
  if(params.timerUntilJamChecks.goneOff() && antiJamEnabled &&
     mtr->getVelocity() < params.jamVelocity) {
    state = IntakeState::Jammed;
  }
}

void Intake::unjamming() {
  mtr->moveVoltage(-12);
  wait(params.timeUntilUnjammed);
  forceIntake(returnState);
}

void Intake::sorting() {
  mtr->moveVoltage(12);
  Timer timeout{params.generalTimeout};
  while(colorSensor->getColor() == sortOutColor && !timeout.goneOff()) {
    if(params.timerUntilJamChecks.goneOff() && antiJamEnabled &&
       mtr->getVelocity() < params.jamVelocity) {
      state = IntakeState::Jammed;
      return;
    }
    wait(ColorSensor::refreshRate);
  }
  // Short delay after seems to provide minor advantage.
  wait();
  mtr->moveVoltage(-12);
  wait(params.sortThrowTime);
  forceIntake(returnState);
}

void Intake::forceIntake(const IntakeState newState) {
  // It should at least be at a standstill before checks occur. This is to
  // prevent false jams if going from outtaking to inttaking quickly.
  if((state != IntakeState::Intaking && state != IntakeState::Indexing) ||
     mtr->getVelocity() < 0_rpm) {
    params.timerUntilJamChecks.resetAlarm();
  }
  state = newState;
  returnState = newState;
}

TASK_DEFINITIONS_FOR(Intake) {
  START_TASK("Intake State Machine")
  while(true) {
    switch(state) {
      case IntakeState::Idle: mtr->brake(); break;
      case IntakeState::Indexing:
      case IntakeState::Intaking: intaking(); break;
      case IntakeState::Outtaking: mtr->moveVoltage(-12); break;
      case IntakeState::Jammed: unjamming(); break;
      case IntakeState::Sorting: sorting(); break;
    }
    wait(10_ms);
  }
  END_TASK
}
} // namespace atum