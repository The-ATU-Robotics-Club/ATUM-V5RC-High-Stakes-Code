#include "piston.hpp"

namespace atum {
Piston::Piston(const std::uint8_t port,
               const bool iReversed,
               const bool startExtended,
               const Logger::Level loggerLevel) :
    piston{port, iReversed ? !startExtended : startExtended},
    reversed{iReversed},
    logger{loggerLevel} {
  logger.debug("Piston on port " +
               std::to_string(std::get<1>(piston.get_port())) +
               " has been constructed.");
}

Piston::Piston(const ADIExtenderPort &port,
               const bool iReversed,
               const bool startExtended,
               const Logger::Level loggerLevel) :
    piston{port(), iReversed ? !startExtended : startExtended},
    reversed{iReversed},
    logger{loggerLevel} {
  logger.debug("Piston on port " +
               std::to_string(std::get<1>(piston.get_port())) +
               " has been constructed.");
}

void Piston::extend() {
  // Don't actuate pistons if disabled. 
  if(pros::competition::is_disabled()) {
    return;
  }
  if(reversed) {
    piston.retract();
  } else {
    piston.extend();
  }
}

void Piston::retract() {
  // Don't actuate pistons if disabled. 
  if(pros::competition::is_disabled()) {
    return;
  }
  if(reversed) {
    piston.extend();
  } else {
    piston.retract();
  }
}

void Piston::toggle() {
  // Don't actuate pistons if disabled. 
  if(pros::competition::is_disabled()) {
    return;
  }
  piston.toggle();
}

bool Piston::isExtended() {
  if(reversed) {
    return !piston.is_extended();
  }
  return piston.is_extended();
}
} // namespace atum