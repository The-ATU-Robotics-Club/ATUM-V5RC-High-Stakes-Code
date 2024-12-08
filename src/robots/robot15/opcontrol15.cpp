#include "robot15.hpp"

namespace atum {
void Robot15::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().y};
    const double turn{remote.getRStick().x};
    leftMotors.moveVoltage(forward + turn);
    rightMotors.moveVoltage(forward - turn);

    switch(remote.getRTrigger()) {
      case -1: intake.moveVoltage(-12); break;
      case 1: intake.moveVoltage(12); break;
      default: intake.brake(); break;
    }

    switch(remote.getLTrigger()) {
      case -1: ladybrownArm.moveVoltage(-6); break;
      case 1: ladybrownArm.moveVoltage(6); break;
      default: ladybrownArm.brake(); break;
    }

    if(remote.getPress(Remote::Button::A)) {
      goalClamp.toggle();
    }

    if(remote.getPress(Remote::Button::X)) {
      ladybrownWrist.toggle();
    }

    wait(10_ms);
  }
}
} // namespace atum