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
     state == IntakeState::FinishedLoading) {
    return;
  }
  state = IntakeState::Loading;
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

// void Intake::intaking() {
//   if(ladybrown->mayConflictWithIntake()) {
//     params.timerUntilJamChecks.setTime();
//     if(state != IntakeState::FinishedLoading) {
//       finishLoading();
//     }
//     return;
//   } else if(state == IntakeState::FinishedLoading) {
//     state = IntakeState::Loading;
//     return;
//   }
//   if(shouldSort()) {
//     state = IntakeState::Sorting;
//     return;
//   }
//   if(shouldIndex()) {
//     if(colorSensor->getColor() != ColorSensor::Color::None) {
//       params.timerUntilJamChecks.setTime();
//       motor->brake();
//       return;
//     }
//   }
//   if(params.timerUntilJamChecks.goneOff() &&
//      motor->getVelocity() < params.jamVelocity) {
//     state = IntakeState::Jammed;
//   }
//   if(state == IntakeState::Indexing ||
//      (state == IntakeState::Loading && ladybrown->hasRing())) {
//     motor->moveVoltage(params.indexingVoltage);
//   } else {
//     motor->moveVoltage(params.intakingVoltage);
//   }
// }

// void Intake::unjamming() {
//   motor->moveVoltage(-12);
//   wait(params.timeUntilUnjammed);
//   if(returnState == IntakeState::Loading) {
//     ladybrown->prepare();
//   }
//   forceIntake(returnState);
// }

// void Intake::sorting() {
//   if(ladybrown->getClosestNamedPosition() == LadybrownState::Loading) {
//     ladybrown->prepare();
//   }
//   motor->moveVoltage(12);
//   Timer timeout{params.generalTimeout};
//   while(shouldSort() && !timeout.goneOff()) {
//     if(params.timerUntilJamChecks.goneOff() &&
//        motor->getVelocity() < params.jamVelocity) {
//       state = IntakeState::Jammed;
//       return;
//     }
//     wait(ColorSensor::refreshRate);
//   }
//   // Short delay after seems to provide minor advantage.
//   wait();
//   motor->moveVoltage(-12);
//   wait(params.sortThrowTime);
//   forceIntake(returnState);
// }

// void Intake::finishLoading() {
//   motor->moveVoltage(params.intakingVoltage);
//   wait(params.pressLoadTime);
//   motor->moveVoltage(-12);
//   wait(params.finishLoadingTime);
//   motor->brake();
//   ladybrown->prepare();
//   wait(params.stopTime);
//   state = IntakeState::FinishedLoading;
// }

// void Intake::forceIntake(const IntakeState newState) {
//   // It should at least be at a standstill before checks occur. This is to
//   // prevent false jams if going from outtaking to intaking quickly.
//   if((state != IntakeState::Intaking && state != IntakeState::Indexing &&
//       state != IntakeState::Loading) ||
//      motor->getVelocity() < 0_rpm) {
//     params.timerUntilJamChecks.setTime();
//   }
//   state = newState;
//   returnState = newState;
// }

// bool Intake::shouldIndex() const {
//   bool ladybrownNotInPosition{state == IntakeState::Loading &&
//                               ladybrown->getClosestNamedPosition() !=
//                                   LadybrownState::Loading};
//   // If indexing or loading while the ladybrown isn't ready, index.
//   return colorSensor->check() &&
//          (state == IntakeState::Indexing || ladybrownNotInPosition);
// }

// bool Intake::shouldSort() const {
//   return colorSensor->check() && sortOutColor != ColorSensor::Color::None &&
//          colorSensor->getColor() == sortOutColor;
// }

TASK_DEFINITIONS_FOR(Intake) {
  START_TASK("Intake State Machine")
  while(true) {
    switch(state) {
      case IntakeState::Idle: voltage = 0.0; break;
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
        voltage = 5.5;
        wait(params.pressLoadTime);
        voltage = -params.indexingVoltage;
        Timer timeout{params.generalTimeout};
        motor->resetPosition();
        while(motor->getPosition() > -params.backupFromLoad &&
              !timeout.goneOff()) {
          wait();
        }
        state = IntakeState::FinishedLoading;
      } break;
      case IntakeState::FinishedLoading: voltage = 0.0; break;
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
       state == IntakeState::PressLoading ||
       state == IntakeState::FinishedLoading || !voltage) {
      params.timerUntilJamChecks.setTime();
    } else if(params.timerUntilJamChecks.goneOff() &&
              motor->getVelocity() < params.jamVelocity) {
      motor->moveVoltage(-12.0);
      wait(params.timeUntilUnjammed);
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
      while(colorSensor->getColor() == sortOutColor && !timeout.goneOff()) {
        wait();
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