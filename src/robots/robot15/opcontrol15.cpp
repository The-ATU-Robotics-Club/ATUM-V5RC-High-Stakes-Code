#include "robot15.hpp"

namespace atum {
void Robot15::opcontrol() {
  while(true) {
    const double forward{remote.getLStick().second};
    const double turn{remote.getRStick().first};
    leftMotors.move_voltage(forward + turn);
    rightMotors.move_voltage(forward - turn);
    wait(10_ms);
  }
}
}