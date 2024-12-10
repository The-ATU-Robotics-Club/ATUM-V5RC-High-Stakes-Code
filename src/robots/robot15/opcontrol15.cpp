#include "robot15.hpp"

namespace atum {
void Robot15::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().y};
    const double turn{remote.getRStick().x};
    leftMotors.moveVoltage(forward + turn);
    rightMotors.moveVoltage(forward - turn);

    switch(remote.getRTrigger()) {
      case -1: intake->outtake(); break;
      case 1: intake->intake(); break;
      default: intake->stop(); break;
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

    remote.print(
        0,
        "Brain: " +
            std::to_string(static_cast<int>(pros::battery::get_capacity())) +
            "%");
    remote.print(1, "Remote: " + std::to_string(remote.getBattery()) + "%");

    wait(10_ms);
  }
}
} // namespace atum