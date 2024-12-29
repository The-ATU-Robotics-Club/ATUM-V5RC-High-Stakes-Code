#include "intake.hpp"

namespace atum {
Intake::Intake(std::unique_ptr<Motor> iMtr,
               std::unique_ptr<ColorSensor> iColorSensor,
               Ladybrown *iLadybrown,
               const Parameters &iParams,
               const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    mtr{std::move(iMtr)},
    colorSensor{std::move(iColorSensor)},
    ladybrown{iLadybrown},
    logger{loggerLevel},
    params{iParams} {
  logger.info("Intake is constructed!");
  stop();
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

void Intake::load() {
  if(state == IntakeState::Jammed || state == IntakeState::Sorting ||
     state == IntakeState::FinishedLoading) {
    return;
  }
  forceIntake(IntakeState::Loading);
}

void Intake::outtake() {
  state = IntakeState::Outtaking;
}

void Intake::stop() {
  state = IntakeState::Idle;
}

void Intake::setSortOutColor(const ColorSensor::Color iSortOutColor) {
  sortOutColor = iSortOutColor;
}

ColorSensor::Color Intake::getSortOutColor() const {
  return sortOutColor;
}

void Intake::intaking() {
  if(ladybrown->mayConflictWithIntake()) {
    params.timerUntilJamChecks.resetAlarm();
    if(state != IntakeState::FinishedLoading) {
      finishLoading();
    }
    return;
  } else if(state == IntakeState::FinishedLoading) {
    state = IntakeState::Loading;
    return;
  }
  if(shouldSort()) {
    state = IntakeState::Sorting;
    return;
  }
  // If indexing or loading while the ladybrown isn't ready, index.
  if(shouldIndex()) {
    if(colorSensor->getColor() != ColorSensor::Color::None) {
      params.timerUntilJamChecks.resetAlarm();
      mtr->brake();
      return;
    }
  }
  if(params.timerUntilJamChecks.goneOff() &&
     mtr->getVelocity() < params.jamVelocity) {
    state = IntakeState::Jammed;
  }
  mtr->moveVoltage(12);
}

void Intake::unjamming() {
  mtr->moveVoltage(-12);
  wait(params.timeUntilUnjammed);
  forceIntake(returnState);
}

void Intake::sorting() {
  if(ladybrown->getClosestNamedPosition() == LadybrownState::Loading) {
    ladybrown->prepare();
  }
  mtr->moveVoltage(12);
  Timer timeout{params.generalTimeout};
  while(colorSensor->getColor() == sortOutColor && !timeout.goneOff()) {
    if(params.timerUntilJamChecks.goneOff() &&
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

void Intake::finishLoading() {
  mtr->moveVoltage(-12);
  wait(params.finishLoadingTime);
  state = IntakeState::FinishedLoading;
  ladybrown->prepare();
}

void Intake::forceIntake(const IntakeState newState) {
  // It should at least be at a standstill before checks occur. This is to
  // prevent false jams if going from outtaking to intaking quickly.
  if((state != IntakeState::Intaking && state != IntakeState::Indexing &&
      state != IntakeState::Loading) ||
     mtr->getVelocity() < 0_rpm) {
    params.timerUntilJamChecks.resetAlarm();
  }
  state = newState;
  returnState = newState;
}

bool Intake::shouldIndex() const {
  bool ladybrownNotInPosition{state == IntakeState::Loading &&
                              ladybrown->getClosestNamedPosition() !=
                                  LadybrownState::Loading};
  return state == IntakeState::Indexing || ladybrownNotInPosition;
}

bool Intake::shouldSort() const {
  return sortOutColor != ColorSensor::Color::None &&
         colorSensor->getColor() == sortOutColor;
}

TASK_DEFINITIONS_FOR(Intake) {
  START_TASK("Intake State Machine")
  while(true) {
    switch(state) {
      case IntakeState::Idle: mtr->brake(); break;
      case IntakeState::Loading:
        if(ladybrown->getClosestNamedPosition() != LadybrownState::Loading &&
           !ladybrown->hasRing()) {
          ladybrown->load();
        }
        // fall through
      case IntakeState::FinishedLoading:
      case IntakeState::Indexing:
      case IntakeState::Intaking: intaking(); break;
      case IntakeState::Outtaking: mtr->moveVoltage(-12); break;
      case IntakeState::Jammed: unjamming(); break;
      case IntakeState::Sorting: sorting(); break;
    }
    wait();
  }
  END_TASK
}
} // namespace atum