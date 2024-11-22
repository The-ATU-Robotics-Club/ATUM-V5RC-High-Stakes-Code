#include "robotPrototype.hpp"

namespace atum {
void RobotPrototype::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().second};
    const double turn{remote.getRStick().first};
    leftMotors.moveVoltage(forward + turn);
    rightMotors.moveVoltage(forward - turn);
    wait(10_ms);
  }
}
}