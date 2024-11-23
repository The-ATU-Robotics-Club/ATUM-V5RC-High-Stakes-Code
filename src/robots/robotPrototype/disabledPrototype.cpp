#include "robotPrototype.hpp"

namespace atum {
RobotPrototype::RobotPrototype() : Robot{this} {
    climbMotors.setBrakeMode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
}

void RobotPrototype::disabled() {}
} // namespace atum