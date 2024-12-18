#include "drive.hpp"

namespace atum {
Drive::Drive(std::unique_ptr<pros::MotorGroup> iLeft,
             std::unique_ptr<pros::MotorGroup> iRight,
             const Geometry &iGeometry,
             std::unique_ptr<Tracker> iTracker,
             std::unique_ptr<Logger> iLogger) :
    left{std::move(iLeft)},
    right{std::move(iRight)},
    geometry{iGeometry},
    tracker{std::move(iTracker)},
    logger{std::move(iLogger)} {
  if(logger) logger->info("Drive constructed!");
}

void Drive::tank(const double leftVoltage, const double rightVoltage) {
  if(logger && (abs(leftVoltage) > Motor::maxVoltage ||
                abs(rightVoltage) > Motor::maxVoltage))
    logger->warn("Input voltage exceeds motor range (>12V).");
  if(!leftVoltage && !rightVoltage) {
    left->move_velocity(0);
    right->move_velocity(0);
  } else {
    left->move_voltage(leftVoltage);
    right->move_voltage(rightVoltage);
  }
}

void Drive::arcade(const double forwardVoltage, const double turnVoltage) {
  const double left{std::clamp(
      forwardVoltage + turnVoltage, -Motor::maxVoltage, Motor::maxVoltage)};
  const double right{std::clamp(
      forwardVoltage - turnVoltage, -Motor::maxVoltage, Motor::maxVoltage)};
  tank(left, right);
}

void Drive::setPosition(const Pose &iPosition) {
  if(!tracker) {
    if(logger) logger->error("No tracker provided!");
    return;
  }
  tracker->setPosition(iPosition);
}

Pose Drive::getPosition() const {
  if(!tracker) {
    if(logger) logger->error("No tracker provided!");
    return Pose{};
  }
  return tracker->getPosition();
}

meters_per_second_t Drive::getLinearVelocity() const {
  const auto [lRPM, rRPM] = getLRVelocity();
  const double rpm{std::abs(lRPM + rRPM) / 2.0};
  return rpm * geometry.wheelCircumference / 60.0_s;
}

radians_per_second_t Drive::getAngularVelocity() const {
  const auto [lRPM, rRPM] = getLRVelocity();
  const meters_per_second_t lVel{lRPM * geometry.wheelCircumference / 60.0_s};
  const meters_per_second_t rVel{rRPM * geometry.wheelCircumference / 60.0_s};
  return radians_per_second_t{getValueAs<meters_per_second_t>(lVel - rVel) /
                              getValueAs<meter_t>(geometry.driveWidth)};
}

std::pair<double, double> Drive::getLRVelocity() const {
  const double lAvg{average(left->get_actual_velocity_all())};
  const double rAvg{average(right->get_actual_velocity_all())};
  return std::make_pair(lAvg * geometry.gearRatio, rAvg * geometry.gearRatio);
}

std::pair<double, double> Drive::toLRVelocity(const double v,
                                              const double w) const {
  const double angularAdjustment{w * getValueAs<meter_t>(geometry.driveWidth) /
                                 2.0};
  const double denom{getValueAs<meter_t>(geometry.wheelCircumference)};
  return std::make_pair(60.0 * (v + angularAdjustment) / denom,
                        60.0 * (v - angularAdjustment) / denom);
}

void Drive::setBrakeMode(const pros::v5::MotorBrake brakeMode) {
  left->set_brake_mode_all(brakeMode);
  right->set_brake_mode_all(brakeMode);
}

pros::v5::MotorBrake Drive::getBrakeMode() const {
  return left->get_brake_mode();
}

Drive::Geometry Drive::getGeometry() const {
  return geometry;
}
} // namespace atum