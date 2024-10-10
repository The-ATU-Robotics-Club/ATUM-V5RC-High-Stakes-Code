#include "robot.hpp"

namespace atum {
Robot::Robot() : Task{{"Robot"}} {
  start();
}

void Robot::schedule(const ScheduledAction &scheduledAction) {
  timedScheduledAction = make_pair(time(), scheduledAction);
}

void Robot::deschedule() {
  timedScheduledAction = {};
}

void Robot::taskFn1() {
  while(true) {
    wait(50_ms);
    if(!timedScheduledAction) continue;
    auto [startTime, scheduledAction] = timedScheduledAction.value();
    if(scheduledAction.timeout &&
       time() - startTime > scheduledAction.timeout) {
      if(scheduledAction.actOnTimeout && !pros::competition::is_disabled())
        scheduledAction.action();
      deschedule();
      continue;
    }
    if(scheduledAction.condition() && !pros::competition::is_disabled()) {
      scheduledAction.action();
      deschedule();
      continue;
    }
  }
}
} // namespace atum