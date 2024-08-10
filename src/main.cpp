#include "main.h"

std::unique_ptr<atum::Robot> robot;

void initialize() {
  std::unique_ptr<atum::Logger> logger =
      atum::Logger::makeLog(atum::Logger::LoggerLevel::Debug);
  if(atum::fileExists("/usd/15in.txt"))
    robot = std::make_unique<atum::Robot15In>(std::move(logger));
  else if(atum::fileExists("/usd/24in.txt"))
    robot = std::make_unique<atum::Robot24In>(std::move(logger));
  else
    robot = std::make_unique<atum::RobotDescore>(std::move(logger));
}

void disabled() {
  atum::GUI::autonSelector();
  robot->disabled();
}

void competition_initialize() {
  atum::GUI::autonSelector();
  robot->disabled();
}

void autonomous() {
  robot->autonomous();
}

void opcontrol() {
  robot->opcontrol();
}
