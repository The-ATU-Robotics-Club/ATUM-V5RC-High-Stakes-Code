#include "schedule.hpp"

namespace atum {
const std::function<void()> Schedule::doNothing{[]() {}};
const Condition Schedule::neverMet{[]() { return false; }};

Schedule::Schedule(const Item &iItem, const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    item{iItem},
    logger{loggerLevel} {
  startBackgroundTasks();
  logger.debug("The item \"" + item.name + "\" has been scheduled.");
}

TASK_DEFINITIONS_FOR(Schedule) {
  START_TASK("Scheduled Item")
  const uint8_t initialStatus{pros::competition::get_status()};
  Timer timer{item.timeout};
  while(pros::competition::get_status() == initialStatus && !item.condition() &&
        !timer.goneOff()) {
    wait();
  }
  if(pros::competition::get_status() != initialStatus) {
    logger.debug("Scheduled item was interrupted.");
    return;
  }
  if(item.timeout && timer.goneOff()) {
    item.todoTimeout();
    logger.debug("The scheduled item \"" + item.name + "\" has timed out.");
    return;
  }
  item.todo();
  logger.debug("The scheduled item \"" + item.name + "\" is finished.");
  END_TASK
}
} // namespace atum