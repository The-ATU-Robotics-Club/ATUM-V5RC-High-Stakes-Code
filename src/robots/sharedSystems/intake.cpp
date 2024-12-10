#include "intake.hpp"

namespace atum {
Intake::Intake(std::unique_ptr<Motor> iMtr,
               const Parameters &iParams,
               const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    mtr{std::move(iMtr)},
    logger{loggerLevel},
    params{iParams} {
  logger.info("Intake is constructed!");
}

void Intake::intake() {
  if(state == IntakeState::Jammed) {
    return;
  }
  if(state != IntakeState::Intaking) {
    params.timerUntilJamChecks.resetAlarm();
  }
  state = IntakeState::Intaking;
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

TASK_DEFINITIONS_FOR(Intake) {
  START_TASK("Intake State Machine")
  while(true) {
    switch(state) {
      case IntakeState::Idle: mtr->brake(); break;
      case IntakeState::Intaking:
        mtr->moveVoltage(12);
        if(params.timerUntilJamChecks.goneOff() &&
           mtr->getVelocity() < params.jamVelocity) {
          state = IntakeState::Jammed;
        }
        break;
      case IntakeState::Outtaking: mtr->moveVoltage(-12); break;
      case IntakeState::Jammed:
        mtr->moveVoltage(-12);
        wait(params.timeUntilUnjammed);
        state = IntakeState::Idle;
        break;
    }
    wait(10_ms);
  }
  END_TASK
}
} // namespace atum