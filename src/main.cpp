#include "main.h"

static constexpr int prototypeID{0x9e344900};
static constexpr int a15ID{0x64824900};

#ifndef BRAIN_ID
#define BRAIN_ID 0
#endif

std::unique_ptr<atum::Robot> robot;
atum::Logger logger{};

void initialize() {
  switch(BRAIN_ID) {
    case prototypeID:
      robot = std::make_unique<atum::RobotPrototype>(
          std::initializer_list<std::int8_t>{},
          std::initializer_list<std::int8_t>{});
      break;

    case a15ID:
      robot = std::make_unique<atum::RobotPrototype>(
          std::initializer_list<std::int8_t>{},
          std::initializer_list<std::int8_t>{});
      break;
  }
  atum::GUI::startLoading(robot->getRoutineNames());
  atum::wait(0.5_s);
  atum::GUI::finishLoading();
  logger.info("Initialization finished!");
}

void disabled() {
  robot->disabled();
}

void competition_initialize() {
  robot->disabled();
}

void autonomous() {
  robot->autonomous();
}

void opcontrol() {
  logger.info("Opcontrol has started!");
  robot->opcontrol();
}
