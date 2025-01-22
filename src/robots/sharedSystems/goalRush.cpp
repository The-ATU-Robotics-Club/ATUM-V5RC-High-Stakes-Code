#include "goalrush.hpp"

namespace atum {

goalrush::goalrush(std::unique_ptr<Piston> goalRush,
 std::unique_ptr<Piston> goalRushClamp, const Logger::Level loggerLevel) :

 goalRush{std::move(goalRush)},
 goalRushClamp{std::move(goalRushClamp)},
 logger{loggerLevel} {
 logger.info("Goal rush is constructed!");
 }


void goalrush::armExtend() {
  goalRush->extend();
}

void goalrush::armRetract() {
  goalRush->retract();
}

void goalrush::armToggle() {
  goalRush->toggle();
}

//bool goalrush::armExtended() const {
//}

void goalrush::grab() {
  goalRushClamp->extend();
}

void goalrush::release() {
  goalRushClamp->retract();
}

void goalrush::rushClampToggle() {
  goalRushClamp->toggle();
}
}