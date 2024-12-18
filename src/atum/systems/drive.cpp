#include "drive.hpp"

namespace atum {
Drive::Drive(std::unique_ptr<Motor> iLeft,
             std::unique_ptr<Motor> iRight,
             std::unique_ptr<Tracker> iTracker,
             const Logger::Level loggerLevel) :
    left{std::move(iLeft)},
    right{std::move(iRight)},
    tracker{std::move(iTracker)},
    logger{loggerLevel} {
  if(!left) {
    logger.error("The left side drive motors were not provided!");
  }
  if(!right) {
    logger.error("The right side drive motors were not provided!");
  }
  if(!tracker) {
    logger.error("No tracker provided for the drive!");
    return;
  }
  logger.info("Drive constructed!");
}

void Drive::tank(const double leftVoltage, const double rightVoltage) {
  if(abs(leftVoltage) > Motor::maxVoltage ||
     abs(rightVoltage) > Motor::maxVoltage) {
    logger.warn("Input voltage exceeds motor range (>12V).");
  }
  if(!leftVoltage && !rightVoltage) {
    left->brake();
    right->brake();
  } else {
    left->moveVoltage(leftVoltage);
    right->moveVoltage(rightVoltage);
  }
}

void Drive::arcade(const double forwardVoltage, const double turnVoltage) {
  const double left{std::clamp(
      forwardVoltage + turnVoltage, -Motor::maxVoltage, Motor::maxVoltage)};
  const double right{std::clamp(
      forwardVoltage - turnVoltage, -Motor::maxVoltage, Motor::maxVoltage)};
  tank(left, right);
}

void Drive::setPose(const Pose &iPose) {
  tracker->setPose(iPose);
}

Pose Drive::getPose() const {
  return tracker->getPose();
}

void Drive::setBrakeMode(const pros::v5::MotorBrake brakeMode) {
  left->setBrakeMode(brakeMode);
  right->setBrakeMode(brakeMode);
}

pros::v5::MotorBrake Drive::getBrakeMode() const {
  return left->getBrakeMode();
}
} // namespace atum