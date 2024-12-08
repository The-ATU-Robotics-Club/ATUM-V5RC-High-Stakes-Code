#include "robot15.hpp"

namespace atum {
Robot15::Robot15() : Robot{this} {
    ladybrownArm.setBrakeMode(pros::v5::MotorBrake::hold);
}

void Robot15::disabled() {}
} // namespace atum