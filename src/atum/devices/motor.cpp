#include "motor.hpp"

namespace atum {
Motor::Motor(const std::vector<std::int8_t> ports,
             const pros::v5::MotorGears gearset,
             const Logger::LoggerLevel loggerLevel) {}

void Motor::moveVelocity(const revolutions_per_minute_t velocity) {}

void Motor::moveVoltage(const double voltage) {}

degree_t Motor::getPosition() const {
    std::vector<degree_t> positions;
    for(std::size_t i{0}; i < motors.size(); i++) {
        const degree_t position{motors[i]->get_position()};
        positions.push_back(position);
    }
  return average(positions);
}

revolutions_per_minute_t Motor::getVelocity() const {}

revolutions_per_minute_t Motor::getTargetVelocity() const {
  const std::int32_t targetVelocity{motors[0]->get_target_velocity()};
  return revolutions_per_minute_t{targetVelocity};
}

std::int32_t Motor::getCurrentDraw() const {
    
}

double Motor::getEfficiency() const {}

double Motor::getPower() const {}

double Motor::getTemperature() const {}

double Motor::getTorque() const {}

std::int32_t Motor::getVoltage() const {}

std::int32_t Motor::getCurrentLimit() const {}

std::int32_t Motor::setBrakeMode(const pros::motor_brake_mode_e_t mode) const {}

std::int32_t Motor::setCurrentLimit(const std::int32_t limit) const {}

void Motor::motorCheck() const {}

} // namespace atum