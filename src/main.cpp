#include "main.h"

#ifndef BRAIN_ID
#define BRAIN_ID 0
#endif

std::unique_ptr<atum::Robot> robot;
atum::Logger logger;

void initialize() {
  atum::GUI::Manager::initialize();
  logger.info("Initialization has started.");
  switch(BRAIN_ID) {
    case atum::Robot15A::ID: robot = std::make_unique<atum::Robot15A>(); break;
    default: robot = std::make_unique<atum::RobotPrototype>(); break;
  }
  atum::wait(0.5_s); // Basic wait for VEX OS to start up.
  robot->disabled(); // Make sure disabled has atleast ran once.
  logger.info("Initialization finished.");
  atum::GUI::Manager::finishLoading();
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
