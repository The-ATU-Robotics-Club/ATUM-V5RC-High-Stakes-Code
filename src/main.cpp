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
      robot = std::make_unique<atum::RobotPrototype>();
      break;

    case a15ID:
      robot = std::make_unique<atum::Robot15>();
      break;
  }
  atum::wait(0.5_s);
  logger.info("Initialization finished!");
  atum::GUI::finishLoading();
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
