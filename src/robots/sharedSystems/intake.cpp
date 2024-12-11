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
  intakeMacro();
}

void Intake::outtake() {
  changeState(IntakeState::Outtaking);
}

void Intake::stop() {
  changeState(IntakeState::Idle);
}

void Intake::setAntiJam(const bool iAntiJamEnabled) {
  antiJamEnabled = iAntiJamEnabled;
}

void Intake::intakeMacro() {
  // It should at least be at a standstill before checks occur. This is to
  // prevent false jams if going from outtaking to inttaking quickly.
  if(state != IntakeState::Intaking || mtr->getVelocity() < 0_rpm) {
    params.timerUntilJamChecks.resetAlarm();
  }
  changeState(IntakeState::Intaking);
}

void Intake::changeState(const IntakeState newState) {
  if(newState != state) {
    previousState = state;
  }
  state = newState;
}

TASK_DEFINITIONS_FOR(Intake) {
  START_TASK("Intake State Machine")
  while(true) {
    switch(state) {
      case IntakeState::Idle: mtr->brake(); break;
      case IntakeState::Intaking:
        mtr->moveVoltage(12);
        if(params.timerUntilJamChecks.goneOff() && antiJamEnabled &&
           mtr->getVelocity() < params.jamVelocity) {
          changeState(IntakeState::Jammed);
        }
        if(sortOutColor != ColorSensor::Color::None &&
           colorSensor->getColor() == sortOutColor) {
          logger.debug("Switching to sorting!");
          changeState(IntakeState::Sorting);
        }
        break;
      case IntakeState::Outtaking: mtr->moveVoltage(-12); break;
      case IntakeState::Jammed:
        mtr->moveVoltage(-12);
        wait(params.timeUntilUnjammed);
        intakeMacro();
        break;
      case IntakeState::Sorting:
        const double initial{mtr->getPosition()};
        mtr->moveVoltage(10);
        while(mtr->getPosition() - initial < 300 ||
              colorSensor->getColor() == sortOutColor) {
          wait(5_ms);
        }
        logger.debug(std::to_string(mtr->getPosition() - initial));
        mtr->moveVoltage(-12);
        wait(0.05_s);
        intakeMacro();
        break;
    }
    wait(10_ms);
  }
  END_TASK
}
} // namespace atum