#include "robot15.hpp"

namespace atum {
Robot15::Robot15(std::initializer_list<std::int8_t> leftPorts,
                 std::initializer_list<std::int8_t> rightPorts) :
    leftMotors{leftPorts}, rightMotors{rightPorts} {}

void Robot15::disabled() {}

void Robot15::opcontrol() {
  while(true) {
    const double leftCmd{remote.getLStick().second};
    leftMotors.move(leftCmd);
    const double rightCmd{remote.getRStick().second};
    rightMotors.move(rightCmd);
    wait(10_ms);
  }
}

void Robot15::autonomous() {
    leftMotors.move(127);
    rightMotors.move(127);
    wait(0.5_s);
    leftMotors.brake();
    rightMotors.brake();
}
} // namespace atum