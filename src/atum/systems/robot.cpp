#include "robot.hpp"

namespace atum {
void Robot::autonomous() {
  // This will later be received from the RoutineSelector.
  const std::size_t routineIndex{0};
  routines[routineIndex]();
}

std::string Robot::getRoutineNames() {
  return routineNames;
}
} // namespace atum