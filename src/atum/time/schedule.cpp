#include "schedule.hpp"

namespace atum {
Schedule::Schedule(const Item &iItem, const Logger::Level loggerLevel) :
    Task{this, loggerLevel}, item{iItem}, logger{loggerLevel} {
  startBackgroundTasks();
}

TASK_DEFINITIONS_FOR(Schedule) {
  START_TASK("Schedule Item")
  Timer timer{item.timeout};
  logger.debug("Timer has: " + std::to_string(timer.goneOff()));
  while(!item.condition() && !timer.goneOff()) {
    wait();
  }
  if(item.timeout && timer.goneOff()) {
    item.todoTimeout();
    return;
  }
  item.todo();
  return;
  END_TASK
}
} // namespace atum