#include "robot15.hpp"

namespace atum {
void Robot15::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().second};
    const double turn{remote.getRStick().first};
    leftMotors.moveVoltage(forward + turn);
    rightMotors.moveVoltage(forward - turn);

    switch(remote.getRTrigger()) {
      case -1: intake.moveVoltage(-12); break;
      case 1: intake.moveVoltage(12); break;
      default: intake.brake(); break;
    }

    switch(remote.getLTrigger()) {
      case -1: ladybrown.moveVoltage(-12); break;
      case 1: ladybrown.moveVoltage(12); break;
      default: ladybrown.brake(); break;
    }

    if(remote.getPress(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A)) {
      goalClamp.toggle();
    }

    wait(10_ms);
  }
}
} // namespace atum