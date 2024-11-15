#include "robot15.hpp"

namespace atum {
Robot15::Robot15(std::initializer_list<std::int8_t> leftPorts,
                 std::initializer_list<std::int8_t> rightPorts) :
    Robot{this}, leftMotors{leftPorts}, rightMotors{rightPorts} {}

void Robot15::disabled() {}
} // namespace atum