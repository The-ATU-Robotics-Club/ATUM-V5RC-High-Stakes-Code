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
  // Remember that these can go off screen! Need to change series ranges!
  atum::LateralProfile::Parameters fourStages{10_mps, 10_mps_sq, 0.1_mps_cb};
  atum::LateralProfile::Parameters fiveStages{0.1_mps, 0.1_mps_sq, 0.1_mps_cb};
  atum::LateralProfile::Parameters sixStages{10_mps, 0.1_mps_sq, 0.1_mps_cb};
  atum::LateralProfile::Parameters allStages{0.1_mps, 0.1_mps_sq, 0.2_mps_cb};
  atum::LateralProfile::Parameters trapezoidal{0.1_mps, 0.1_mps_sq};
  atum::LateralProfile mp{
      0_m, 0.5_m, {0.1_mps, 0.1_mps_sq}, atum::Logger::Level::Debug};
  atum::GUI::Graph::clearAll();
  atum::GUI::Graph::setSeriesRange({0, 0.5}, atum::GUI::SeriesColor::White);
  atum::GUI::Graph::setSeriesRange({0, 0.1}, atum::GUI::SeriesColor::Magenta);
  atum::GUI::Graph::setSeriesRange({-0.1, 0.1}, atum::GUI::SeriesColor::Red);
  atum::GUI::Graph::setSeriesRange({-0.1, 0.1}, atum::GUI::SeriesColor::Blue);
  atum::GUI::Graph::setNumOfPoints(590);
  atum::Timer timer{};
  while(!mp.isDone()) {
    atum::LateralProfile::Point p{mp.getPoint(timer.timeElapsed())};
    atum::GUI::Graph::addValue(atum::getValueAs<meter_t>(p.s),
                               atum::GUI::SeriesColor::White);
    atum::GUI::Graph::addValue(atum::getValueAs<meters_per_second_t>(p.v),
                               atum::GUI::SeriesColor::Magenta);
    atum::GUI::Graph::addValue(
        atum::getValueAs<meters_per_second_squared_t>(p.a),
        atum::GUI::SeriesColor::Red);
    atum::GUI::Graph::addValue(atum::getValueAs<meters_per_second_cubed_t>(p.j),
                               atum::GUI::SeriesColor::Blue);
    atum::wait();
  }
  robot->autonomous();
}

void opcontrol() {
  logger.info("Opcontrol has started.");
  robot->opcontrol();
}
