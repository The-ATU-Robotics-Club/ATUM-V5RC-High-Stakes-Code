#include "robotPrototype.hpp"

namespace atum {
RobotPrototype::RobotPrototype(std::initializer_list<std::int8_t> leftPorts,
                               std::initializer_list<std::int8_t> rightPorts) :
    Robot{this}, leftMotors{leftPorts}, rightMotors{rightPorts} {}

void RobotPrototype::disabled() {
}
} // namespace atum