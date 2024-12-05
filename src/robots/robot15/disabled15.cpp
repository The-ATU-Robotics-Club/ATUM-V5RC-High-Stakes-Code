#include "robot15.hpp"

namespace atum {
Robot15::Robot15() : Robot{this} {
    ladybrown.setBrakeMode(pros::v5::MotorBrake::brake);
}

void Robot15::disabled() {}
} // namespace atum