#include "main.h"

#ifndef BRAIN_ID
#define BRAIN_ID 0
#endif

static constexpr int IDPROTO{0x00000000};

std::unique_ptr<atum::Robot> robot;
atum::Logger logger;

void initialize() {
  atum::GUI::Manager::initialize();
  logger.info("Initialization has started.");
  if constexpr(BRAIN_ID == IDPROTO) {
    robot = std::make_unique<atum::RobotPrototype>();
  } else {
    robot = std::make_unique<atum::RobotClone>(BRAIN_ID);
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
