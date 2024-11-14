#include "main.h"

#ifndef BRAIN_ID
#define BRAIN_ID 0
#endif

std::unique_ptr<atum::Robot> robot;

void initialize() {
  const std::string routines{
    "None\n"
    "Simple Left\n"
    "Simple Right\n"
    "WP Left\n"
    "WP Right\n"
    "Defensive Left\n"
    "Defensive Right\n"
    "Offensive Left"
};
  GUI::startLoading(routines);
  robot = std::make_unique<atum::Robot15>(
      std::initializer_list<std::int8_t>{-11, -12, -13},
      std::initializer_list<std::int8_t>{18, 19, 20});
  atum::wait(0.5_s);
  GUI::finishLoading();
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
