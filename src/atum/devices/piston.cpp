#include "piston.hpp"

namespace atum {
Piston::Piston(const std::uint8_t port,
               const bool startExtended,
               const Logger::Level loggerLevel) :
    piston{port, startExtended},
    logger{loggerLevel} {
  logger.debug("Piston on port " +
               std::to_string(std::get<1>(piston.get_port())) +
               " has been constructed.");
}

Piston::Piston(const ADIExtenderPort &port,
               const bool startExtended,
               const Logger::Level loggerLevel) :
    piston{port(), startExtended},
    logger{loggerLevel} {
  logger.debug("Piston on port " +
               std::to_string(std::get<1>(piston.get_port())) +
               " has been constructed.");
}

void Piston::extend() {
  piston.extend();
}

void Piston::retract() {
  piston.retract();
}

void Piston::toggle() {
  piston.toggle();
}

bool Piston::isExtended() {
  return piston.is_extended();
}
} // namespace atum