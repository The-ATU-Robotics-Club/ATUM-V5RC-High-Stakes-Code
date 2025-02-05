#include "moveTo.hpp"

namespace atum {
MoveTo::MoveTo(Drive *iDrive,
               Turn *iTurn,
               std::unique_ptr<LateralProfileFollower> iFollower,
               std::unique_ptr<PID> iDirectionController,
               const meter_t iTurnToThreshold,
               const Logger::Level loggerLevel) :
    drive{iDrive},
    turn{iTurn},
    follower{std::move(iFollower)},
    directionController{std::move(iDirectionController)},
    turnToThreshold{iTurnToThreshold},
    logger{loggerLevel} {}

void MoveTo::forward(Pose target,
                     const LateralProfile::Parameters &specialParams) {
  turn->toward(target);
  if(flipped) {
    target.flip();
  }
  logger.debug("Moving to " + toString(target) + ".");
  const degree_t initialHeading{drive->getPose().h};
  const degree_t shortestAngle{constrain180(target - initialHeading)};
  follower->startProfile(
      initialHeading, initialHeading + shortestAngle, specialParams);
  while(!follower->isDone() && !interrupted) {
    const Pose state{drive->getPose()};
    const double output{follower->getOutput(state.h, state.omega)};
    drive->arcade(0, output);
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

void MoveTo::reverse(Pose target,
                     const LateralProfile::Parameters &specialParams) {}
} // namespace atum