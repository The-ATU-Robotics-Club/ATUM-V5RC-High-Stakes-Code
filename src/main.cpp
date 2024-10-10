#include "main.h"

std::unique_ptr<atum::Robot> robot;

void initialize() {
  robot = std::make_unique<atum::Robot15>(
      std::initializer_list<std::int8_t>{1, 2, 3},
      std::initializer_list<std::int8_t>{4, 5, 6});
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
  robot->opcontrol();
}
