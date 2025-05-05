#include "robotClone.hpp"
#include "robots/sharedSystems/ladybrown.hpp"


namespace atum {
void RobotClone::opcontrol() {
  setSortToOpposite();
  matchTimer.setTime();
  // Where the first routine should be skills.
  if(GUI::Routines::selectedRoutine() == 0) {
    goalClamp->unclamp();
    intake->setSortOutColor(ColorSensor::Color::None);
  }
  scheduler.schedule({"Rumble at 30s Away",
                      Scheduler::neverMet,
                      [=]() { remote.rumble("---"); },
                      GUI::Routines::selectedRoutine() ? 60_s : 30_s});
  scheduler.schedule({"Drop Goal",
                      Scheduler::neverMet,
                      [=]() {
                        if(useHangControls) {
                          goalClamp->unclamp();
                        }
                      },
                      GUI::Routines::selectedRoutine() ? 89.95_s : 59.95_s});
  drive->setBrakeMode(pros::MotorBrake::coast);
  while(true) {
    const double forward{speedMultiplier * remote.getLStick().y};
    const double turn{remote.getRStick().x};
    drive->arcade(forward,
                  ((turn < 0) ? -1 : 1) * turn * turn / Motor::maxVoltage);

    visualFeedback();

    speedMultiplier = 1.0;
    if(useHangControls) {
      hangControls();
    } else if(useManualControls) {
      manualControls();
    } else if(useLadybrownControls) {
      ladybrownControls();
    } else {
      intakeControls();
    }

    configurationControls();

    if(remote.getPress(Remote::Button::Y)) {
      goalClamp->toggleClamp();

      if(!goalClamp->isClamped()) {
        clampTimer.setTime();
      }
    }
    if(clampTimer.goneOff() && goalClamp->hasGoal()) {
      goalClamp->clamp();
    }

    if(remote.getPress(Remote::Button::Left)) {
      goalRushL->toggle();
    }

    if(remote.getPress(Remote::Button::A)) {
      goalRushR->toggle();
    }

    if(remote.getHold(Remote::Button::Up) &&
       remote.getHold(Remote::Button::Down)) {
      GUI::Manager::easteregg();
    }

    wait(20_ms);
  }
}

void RobotClone::visualFeedback() {
  remote.print(
      0, std::string{"CLAMP: "} + (goalClamp->isClamped() ? "Down" : "Up"));
  remote.print(1, "SORT: " + toString(intake->getSortOutColor()));
}

void RobotClone::manualControls() {
  remote.print(2, "MODE: Manual");

  switch(remote.getLTrigger()) {
    case -1: ladybrown->retract(); break;
    case 1: ladybrown->extend(); break;
    default: ladybrown->stop(); break;
  }

  switch(remote.getRTrigger()) {
    case -1: intake->outtake(); break;
    case 1: intake->intake(); break;
    default: intake->stop(); break;
  }
}

void RobotClone::ladybrownControls() {
  remote.print(2, "MODE: Ladybrown");

  if(remote.getHold(Remote::Button::L2) &&
     ladybrown->getState() == LadybrownState::Loading) {
    useLadybrownControls = false;
  }
  if(intake->getState() == IntakeState::Pressed &&
     remote.getPress(Remote::Button::R2)) {
    ladybrown->prepare();
  } else {
    switch(remote.getRTrigger()) {
      case -1:
        if(ladybrown->getState() != LadybrownState::Preparing &&
           ladybrown->getState() != LadybrownState::Packing) {
          intake->outtake();
        }
        break;
      case 1:
        if(ladybrown->getState() != LadybrownState::Preparing &&
           ladybrown->getState() != LadybrownState::Packing) {
          intake->load();
        }
        break;
      default:
        intake->stop();
        switch(remote.getLTrigger()) {
          case -1:
            if(ladybrown->getPosition() >= 45_deg) {
              ladybrown->retract();
            } else {
              ladybrown->load();
            }
            break;
          case 1: ladybrown->extend(); break;
          default: ladybrown->stop(); break;
        }
        break;
    }
  }
}

void RobotClone::intakeControls() {
  remote.print(2, "MODE: Intake");

  ladybrown->rest();

  switch(remote.getRTrigger()) {
    case -1: intake->outtake(); break;
    case 1:
      if(goalClamp->isClamped() || remote.getLTrigger() == -1) {
        intake->intake();
      } else {
        intake->index();
      }
      break;
    default: intake->stop(); break;
  }

  if(remote.getPress(Remote::Button::L1)) {
    useLadybrownControls = true;
  }
}

void RobotClone::hangControls() {
  remote.print(2, "MODE: Hang");

  if(remote.getHold(Remote::Button::L1)) {
    if(id == ID15) {
      speedMultiplier = 0.425;
    } else {
      speedMultiplier = 0.35;
    }
    // ladybrown->prepare();
    goalRushL->retract();
    goalRushR->retract();
  } else {
    // ladybrown->fullyExtend();
    goalRushL->extend();
    goalRushR->extend();
  }
  switch(remote.getRTrigger()) {
    case -1: intake->outtake(); break;
    case 1: intake->intake(); break;
    default: intake->stop(); break;
  }
}

void RobotClone::configurationControls() {
  if(remote.getPress(Remote::Button::X)) {
    useManualControls = !useManualControls;
    useHangControls = false; // Hang controls have the lowest precedence.
  }

  if(remote.getPress(Remote::Button::Right)) {
    useLadybrownControls = !useLadybrownControls;
    useHangControls = false; // Hang controls have the lowest precedence.
  }

  if(remote.getPress(Remote::Button::Up)) {
    useHangControls = !useHangControls;
  }

  if(remote.getPress(Remote::Button::Down)) {
    const ColorSensor::Color currentColor{intake->getSortOutColor()};
    const ColorSensor::Color nextColor{(static_cast<int>(currentColor) + 1) %
                                       3};
    intake->setSortOutColor(nextColor);
  }

  if(remote.getPress(Remote::Button::B)) {
    kaboomer->toggle();
  }
}
} // namespace atum