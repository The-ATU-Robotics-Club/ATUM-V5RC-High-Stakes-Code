#include "robotClone.hpp"


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
  drive->setPose({-2_tile, 2_tile, 90_deg});
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
    }

    if(remote.getPress(Remote::Button::Left)) {
      goalRush1->toggle();
    }

    if(remote.getPress(Remote::Button::A)) {
      goalRush2->toggle();
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

  /*
  L1 brings ladybrown into loading from rest and go to ladybrown mode.
  When loading, if there's no ring, pressing L2 will make the ladybrown rest and
  go to intake mode. If trying to lower past a certain position, ladybrown will
  go back to loading. L1 and L2 work as expected otherwise.
  */

  switch(remote.getRTrigger()) {
    case -1: intake->outtake(); break;
    case 1: intake->load(); break;
    default:
      intake->stop();
      switch(remote.getLTrigger()) {
        case -1: ladybrown->retract(); break;
        case 1: ladybrown->extend(); break;
        default: ladybrown->stop(); break;
      }
      break;
  }

  if(goalClamp->hasGoal() && !recentlyUnclamped) {
    goalClamp->clamp();
    recentlyUnclamped = true;
  } else if(!goalClamp->hasGoal()) {
    recentlyUnclamped = false;
  }
}

void RobotClone::intakeControls() {
  remote.print(2, "MODE: Intake");

  ladybrown->rest();

  switch(remote.getRTrigger()) {
    case -1: intake->outtake(); break;
    case 1:
      if(remote.getHold(Remote::Button::L1) || goalClamp->isClamped()) {
        intake->intake();
      } else {
        intake->index();
      }
      break;
    default: intake->stop(); break;
  }

  if(remote.getHold(Remote::Button::L2)) {
    goalClamp->unclamp();
  } else if(goalClamp->hasGoal() && !recentlyUnclamped) {
    goalClamp->clamp();
    recentlyUnclamped = true;
  } else if(!goalClamp->hasGoal()) {
    recentlyUnclamped = false;
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
    goalRush1->retract();
    goalRush2->retract();
  } else {
    // ladybrown->fullyExtend();
    goalRush1->extend();
    goalRush2->extend();
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
    if(drive->getBrakeMode() == pros::MotorBrake::coast) {
      drive->setBrakeMode(pros::MotorBrake::hold);
    } else {
      drive->setBrakeMode(pros::MotorBrake::coast);
    }
  }
}
} // namespace atum