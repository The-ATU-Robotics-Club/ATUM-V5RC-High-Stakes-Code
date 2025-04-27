#include "scheduler.hpp"


namespace atum {
const std::function<void()> Scheduler::doNothing{[]() {}};
const Condition Scheduler::neverMet{[]() { return false; }};

Scheduler::Item::Item(const std::string &iName,
                      const Condition &iCondition,
                      const std::function<void()> &iTodo,
                      const second_t &iTimeout,
                      const std::optional<std::function<void()>> iTodoTimeout) :
    name{iName},
    condition{iCondition},
    todo{iTodo},
    timeout{iTimeout},
    todoTimeout{iTodoTimeout} {}

Scheduler::Scheduler(const Logger::Level loggerLevel) :
    Task{this, loggerLevel},
    logger{loggerLevel} {
  startBackgroundTasks();
  logger.debug("Scheduler constructed!");
}

Scheduler::~Scheduler() {
  while(scheduled.size()) {
    scheduled.pop();
  }
  logger.debug("Scheduler was interrupted (out of scope).");
}

void Scheduler::schedule(const Scheduler::Item &toSchedule) {
  if(scheduled.size()) {
    logger.warn("Scheduling another item before previous is completed.");
  }
  scheduled.push(toSchedule);
  logger.debug("The item \"" + toSchedule.name + "\" has been scheduled.");
}

TASK_DEFINITIONS_FOR(Scheduler) {
  START_TASK("Scheduler Loop")
  while(true) {
    wait(schedulerLoopDelay);
    if(scheduled.size()) {
      Item item{scheduled.front()};
      const uint8_t initialStatus{pros::competition::get_status()};
      while(pros::competition::get_status() == initialStatus &&
            !item.timeout.goneOff() && !item.condition()) {
        wait(schedulerLoopDelay);
      }
      if(pros::competition::get_status() != initialStatus) {
        while(scheduled.size()) {
          scheduled.pop();
        }
        logger.debug("Scheduled items were interrupted (status change).");
        continue;
      }
      if(item.timeout.goneOff()) {
        item.todoTimeout.value_or(item.todo)();
        scheduled.pop();
        logger.debug("The scheduled item \"" + item.name + "\" has timed out.");
        continue;
      }
      item.todo();
      scheduled.pop();
      logger.debug("The scheduled item \"" + item.name + "\" is finished.");
    }
  }
  END_TASK
}
} // namespace atum