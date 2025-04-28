#include "goalRush.hpp"

namespace atum {
GoalRush::GoalRush(std::unique_ptr<Piston> iArm,
                   std::unique_ptr<LimitSwitch> iLimitSwitch,
                   const Logger::Level loggerLevel) :
    arm{std::move(iArm)},
    limitSwitch{std::move(iLimitSwitch)},
    logger{loggerLevel} {
  logger.info("Goal rush is constructed!");
}

void GoalRush::extend() {
  arm->extend();
}

void GoalRush::retract() {
  arm->retract();
}

void GoalRush::toggle() {
  arm->toggle();
}

bool GoalRush::isUp() const {
  return !arm->isExtended();
}

bool GoalRush::hasGoal() const {
  return limitSwitch->isPressed();
}
} // namespace atum