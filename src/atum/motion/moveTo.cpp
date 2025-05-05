#include "moveTo.hpp"


namespace atum {
MoveTo::MoveTo(Drive *iDrive,
               Turn *iTurn,
               const PID &iLateralPID,
               const PID &iDirectionPID,
               const AcceptableDistance &iAcceptable,
               const meter_t iTurnToThreshold,
               const Logger::Level loggerLevel) :
    drive{iDrive},
    turn{iTurn},
    lateralPID{iLateralPID},
    directionPID{iDirectionPID},
    acceptable{iAcceptable},
    turnToThreshold{iTurnToThreshold},
    logger{loggerLevel} {}

void MoveTo::forward(const second_t timeout,
                     Pose target,
                     const double maxVoltage,
                     const bool turnToFirst) {
  if(turnToFirst) {
    turn->toward(timeout, target);
  }
  moveToPoint(timeout, target, maxVoltage, false);
}

void MoveTo::reverse(const second_t timeout,
                     Pose target,
                     const double maxVoltage,
                     const bool turnToFirst) {
  if(turnToFirst) {
    turn->awayFrom(timeout, target);
  }
  moveToPoint(timeout, target, maxVoltage, true);
}

void MoveTo::moveToPoint(const second_t timeout,
                         Pose target,
                         const double maxVoltage,
                         const bool reversed) {
  interrupted = false;
  if(flipped) {
    target.flip();
  }
  logger.debug("Moving to " + toString(target) + ".");
  acceptable.reset(timeout);
  lateralPID.reset();
  directionPID.reset();
  const Pose initialPose{drive->getPose()};
  const degree_t linearH{angle(initialPose, target)};
  const meter_t totalDistance{distance(initialPose, target)};
  while(!acceptable.canAccept() && !interrupted) {
    const Pose pose{drive->getPose()};
    const meter_t traveled{distance(initialPose, pose)};
    double moveOutput{lateralPID.getOutput(getValueAs<meter_t>(traveled),
                                           getValueAs<meter_t>(totalDistance))};
    moveOutput = std::clamp(moveOutput, -maxVoltage, maxVoltage);
    degree_t targetH{(distance(pose, target) < turnToThreshold) ?
                         linearH :
                         angle(pose, target)};
    if(reversed) {
      moveOutput *= -1;
      targetH += 180_deg;
    }
    const double hError{getValueAs<radian_t>(constrain180(targetH - pose.h))};
    const double directionOutput{directionPID.getOutput(hError)};
    drive->arcade(moveOutput, directionOutput);
    acceptable.canAccept(traveled, totalDistance);
    wait();
  }
  drive->brake();
  if(interrupted) {
    logger.debug("Move to was interrupted!");
    interrupted = false;
  } else {
    logger.debug("Move to complete!");
  }
}
} // namespace atum