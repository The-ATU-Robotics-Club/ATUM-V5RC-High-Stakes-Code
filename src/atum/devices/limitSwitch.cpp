#include "limitSwitch.hpp"
#include "atum/time/time.hpp"

namespace atum {
LimitSwitch::LimitSwitch(const std::uint8_t port,
                         const Logger::Level loggerLevel) :
    limitSwitch{port}, logger{loggerLevel} {
  wait(adiCalibrationTime);
}

LimitSwitch::LimitSwitch(const ADIExtenderPort &port,
                         const Logger::Level loggerLevel) :
    limitSwitch{port()}, logger{loggerLevel} {
  wait(adiCalibrationTime);
}

bool LimitSwitch::isPressed() {
  return limitSwitch.get_value();
}

bool LimitSwitch::isNewlyPressed() {
  return limitSwitch.get_new_press();
}
} // namespace atum