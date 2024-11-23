#include "motor.hpp"

namespace atum {
Motor::Motor(const std::vector<std::int8_t> ports,
             const pros::v5::MotorGears gearset,
             const Logger::LoggerLevel loggerLevel) :
    logger{loggerLevel} {
  for(std::int8_t port : ports) {
    motors.push_back(std::make_unique<pros::Motor>(
        abs(port), gearset, pros::v5::MotorEncoderUnits::degrees));
    if(port < 0) motors.back()->set_reversed(true);
  }
  motorCheck();
}

void Motor::moveVelocity(const revolutions_per_minute_t velocity) {
  motorCheck();
  const double rawVelocity{getValueAs<revolutions_per_minute_t>(velocity)};
  for(std::size_t i{0}; i < motors.size(); i++) {
    motors[i]->move_velocity(rawVelocity);
  }
}

void Motor::moveVoltage(double voltage) {
  motorCheck();
  voltage *= 1000;
  for(std::size_t i{0}; i < motors.size(); i++) {
    motors[i]->move_voltage(voltage);
  }
}

void Motor::brake() {
  motorCheck();
  for(std::size_t i{0}; i < motors.size(); i++) {
    motors[i]->brake();
  }
}

degree_t Motor::getPosition() const {
  std::vector<degree_t> positions;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const degree_t position{motors[i]->get_position()};
    positions.push_back(position);
  }
  return average(positions);
}

revolutions_per_minute_t Motor::getVelocity() const {
  std::vector<revolutions_per_minute_t> velocities;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const revolutions_per_minute_t velocity{motors[i]->get_actual_velocity()};
    velocities.push_back(velocity);
  }
  return average(velocities);
}

revolutions_per_minute_t Motor::getTargetVelocity() const {
  const std::int32_t targetVelocity{motors[0]->get_target_velocity()};
  return revolutions_per_minute_t{targetVelocity};
}

std::int32_t Motor::getCurrentDraw() const {
  std::vector<std::int32_t> currents;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const std::int32_t current{motors[i]->get_current_draw()};
    currents.push_back(current);
  }
  return average(currents);
}

double Motor::getEfficiency() const {
  std::vector<double> efficiencies;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const double efficiency{motors[i]->get_efficiency()};
    efficiencies.push_back(efficiency);
  }
  return average(efficiencies);
}

double Motor::getPower() const {
  std::vector<double> powers;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const double power{motors[i]->get_power()};
    powers.push_back(power);
  }
  return average(powers);
}

double Motor::getTemperature() const {
  std::vector<double> temperatures;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const double temperature{motors[i]->get_temperature()};
    temperatures.push_back(temperature);
  }
  return average(temperatures);
}

double Motor::getTorque() const {
  std::vector<double> torques;
  for(std::size_t i{0}; i < motors.size(); i++) {
    const double torque{motors[i]->get_torque()};
    torques.push_back(torque);
  }
  return average(torques);
}

std::int32_t Motor::getVoltage() const {
  return motors[0]->get_voltage();
}

std::int32_t Motor::getCurrentLimit() const {
  return motors[0]->get_current_limit();
}

void Motor::setBrakeMode(const pros::motor_brake_mode_e_t mode) const {
  for(std::size_t i{0}; i < motors.size(); i++) {
    motors[i]->set_brake_mode(mode);
  }
}

void Motor::setCurrentLimit(const std::int32_t limit) const {
  for(std::size_t i{0}; i < motors.size(); i++) {
    motors[i]->set_current_limit(limit);
  }
}

void Motor::motorCheck() {
  for(std::size_t i{0}; i < motors.size(); i++) {
    const std::string port{std::to_string(motors[i]->get_port())};
    if(!motors[i]->is_installed()) {
      logger.error("The motor on port " + port + " is not installed.");
      motors.erase(std::next(motors.begin(), i));
    } else if(motors[i]->is_over_temp()) {
      logger.warn("The motor on port " + port + " is overheating.");
    } 
  }
}

} // namespace atum