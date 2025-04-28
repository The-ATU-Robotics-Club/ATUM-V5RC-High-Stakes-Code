#include "intake.hpp"


namespace atum {
Intake::Intake(std::unique_ptr<Motor> iMotor,
               std::unique_ptr<ColorSensor> iColorSensor,
               Ladybrown *iLadybrown,
               const Parameters &iParams,
               const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    motor{std::move(iMotor)},
    colorSensor{std::move(iColorSensor)},
    ladybrown{iLadybrown},
    logger{loggerLevel},
    params{iParams} {
  motor->setBrakeMode(pros::v5::MotorBrake::brake);
  logger.info("Intake is constructed!");
  stop();
}

void Intake::intake() {
  state = IntakeState::Intaking;
}

void Intake::index() {
  state = IntakeState::Indexing;
}

void Intake::load() {
  if(state == IntakeState::PressLoading ||
     (state == IntakeState::Pressed && !ladybrown->ringInCarriage()) ||
     state == IntakeState::UnpressLoading ||
     state == IntakeState::FinishedLoading) {
    return;
  }
  state = IntakeState::Loading;
}

void Intake::finishLoading() {
  if(state == IntakeState::Pressed) {
    state = IntakeState::UnpressLoading;
  }
}

void Intake::outtake() {
  state = IntakeState::Outtaking;
}

void Intake::stop() {
  if(state == IntakeState::Pressed || state == IntakeState::UnpressLoading) {
    return;
  }
  state = IntakeState::Idle;
}

void Intake::setSortOutColor(const ColorSensor::Color iSortOutColor) {
  sortOutColor = iSortOutColor;
}

ColorSensor::Color Intake::getSortOutColor() const {
  return sortOutColor;
}

ColorSensor::Color Intake::getColor() const {
  if(!colorSensor->check()) {
    return ColorSensor::Color::None;
  }
  return colorSensor->getColor();
}

TASK_DEFINITIONS_FOR(Intake) {
  START_TASK("Intake State Machine")
  while(true) {
    switch(state) {
      case IntakeState::Idle:
      case IntakeState::Pressed:
      case IntakeState::FinishedLoading: voltage = 0.0; break;
      case IntakeState::Loading:
        ladybrown->load();
        voltage = ladybrown->getState() == LadybrownState::Loading ?
                      params.indexingVoltage :
                      0.0;
        if(ladybrown->getState() != LadybrownState::Loading ||
           !ladybrown->ringInCarriage()) {
          break;
        }
        state = IntakeState::PressLoading;
        break;
      case IntakeState::PressLoading: {
        voltage = 7;
        Timer timeout{params.generalTimeout};
        while(state == IntakeState::PressLoading && !timeout.goneOff()) {
          wait();
        }
        if(state == IntakeState::PressLoading) {
          state = IntakeState::Loading;
        }
      } break;
      case IntakeState::UnpressLoading: {
        voltage = -12;
        Timer timeout{params.generalTimeout};
        motor->resetPosition();
        while(motor->getPosition() > -params.backupFromLoad &&
              !timeout.goneOff()) {
          wait();
        }
        voltage = 0;
        wait(100_ms);
        ladybrown->pack();
        state = IntakeState::FinishedLoading;
      } break;
      case IntakeState::Indexing:
        voltage = ladybrown->ringInIndexer() ? 0.0 : params.indexingVoltage;
        break;
      case IntakeState::Intaking: voltage = params.intakingVoltage; break;
      case IntakeState::Outtaking: voltage = -12.0; break;
    }
    wait();
  }
  END_TASK

  START_TASK("Intake Control")
  while(true) {
    wait(); // At the top for continue statements below.

    if(state == IntakeState::Idle || state == IntakeState::Outtaking ||
       state == IntakeState::Pressed || state == IntakeState::UnpressLoading ||
       state == IntakeState::FinishedLoading || !voltage) {
      params.timerUntilJamChecks.setTime();
    } else if(params.timerUntilJamChecks.goneOff() &&
              motor->getVelocity() < params.jamVelocity) {
      if(state == IntakeState::PressLoading) {
        state = IntakeState::Pressed;
      } else {
        motor->moveVoltage(-12.0);
        wait(params.timeUntilUnjammed);
      }
      params.timerUntilJamChecks.setTime();
      continue;
    }

    const bool sortingEnabled{sortOutColor != ColorSensor::Color::None};
    const bool sortingState{state == IntakeState::Intaking ||
                            state == IntakeState::Indexing};
    const bool sortSensorsWorking{ladybrown->checkRingDetection() &&
                                  colorSensor->check()};
    if(sortingEnabled && sortingState && sortSensorsWorking &&
       ladybrown->ringInIndexer() && colorSensor->getColor() == sortOutColor) {
      motor->moveVoltage(params.intakingVoltage);
      Timer timeout{params.generalTimeout};
      while(colorSensor->getColor() == sortOutColor &&
            ladybrown->ringInIndexer() && !timeout.goneOff()) {
        wait(5_ms);
      }
      motor->moveVoltage(-12.0);
      wait(params.sortThrowTime);
      // Need to give time for the intake to reverse after throwing before
      // resuming antijam.
      params.timerUntilJamChecks.setTime();
    }

    motor->moveVoltage(voltage);
  }
  END_TASK
}
} // namespace atum