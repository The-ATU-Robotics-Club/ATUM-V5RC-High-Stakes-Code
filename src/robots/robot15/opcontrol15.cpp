#include "robot15.hpp"

namespace atum {
void Robot15::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().second};
    const double turn{remote.getRStick().first};
    leftMotors.move_voltage(forward + turn);
    rightMotors.move_voltage(forward - turn);

    switch(remote.getRTrigger()) {
      case -1:
        intake.move_voltage(-12000);
        break;
      case 1:
        intake.move_voltage(12000);
        break;
      default:
        intake.brake();
        break;
    }
    wait(10_ms);
  }
}
}