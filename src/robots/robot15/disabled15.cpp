#include "robot15.hpp"

namespace atum {
Robot15::Robot15() : Robot{this}, leftMotors{1, 2, 3} {}

void Robot15::disabled() {}
} // namespace atum