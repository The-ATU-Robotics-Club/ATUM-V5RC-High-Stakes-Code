#include "robotPrototype.hpp"

namespace atum {
void RobotPrototype::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().second};
    const double turn{remote.getRStick().first};
    leftMotors.moveVoltage(forward + turn);
    rightMotors.moveVoltage(forward - turn);

    if(remote.getHold(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP)) {
      climbMotors.moveVoltage(12);
    } else if(remote.getHold(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN)) {
      climbMotors.moveVoltage(-12);
    } else {
      climbMotors.brake();
    }

    wait(10_ms);
  }
}
}