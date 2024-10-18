#include "main.h"

#ifndef BRAIN_ID
#define BRAIN_ID 0
#endif

std::unique_ptr<atum::Robot> robot;

void initialize() {
  robot = std::make_unique<atum::Robot15>(
      std::initializer_list<std::int8_t>{-11, -12, -13},
      std::initializer_list<std::int8_t>{18, 19, 20});
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
  std::cout << std::hex << BRAIN_ID << '\n';
  robot->opcontrol();
}
