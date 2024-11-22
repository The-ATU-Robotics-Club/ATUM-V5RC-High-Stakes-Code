#include "main.h"

static constexpr int prototypeID{0x9e344900};
static constexpr int a15ID{0x64824900};

#ifndef BRAIN_ID
#define BRAIN_ID 0
#endif

std::unique_ptr<atum::Robot> robot;
atum::Logger logger{};

void initialize() {
  atum::GUI::initialize();
  logger.info("Initialization has started.");
  switch(BRAIN_ID) {
    case prototypeID: robot = std::make_unique<atum::RobotPrototype>(); break;
    case a15ID: robot = std::make_unique<atum::Robot15>(); break;
  }
  atum::wait(0.5_s); // Temporary wait for testing loading screen.
  logger.info("Initialization finished.");
  atum::GUI::finishLoading();
}

void competition_initialize() {
  // Treat competition_initialize as if disabled.
  disabled();
}

void disabled() {
  logger.info("Robot is disabled.");
  robot->disabled();
}

void autonomous() {
  logger.info("Autonomous has started.");
  robot->autonomous();
}

void opcontrol() {
  logger.info("Opcontrol has started.");
  robot->opcontrol();
}
