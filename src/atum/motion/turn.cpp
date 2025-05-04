#include "atum/depend/units.h"
#include "turn.hpp"


namespace atum {
Turn::Turn(Drive *iDrive,
  const PID &iPID,
  const AcceptableAngle &iAcceptable,
           const Logger::Level loggerLevel) :
    drive{iDrive},
    pid{iPID},
    acceptable{iAcceptable},
    logger{loggerLevel} {}

void Turn::toward(const second_t timeout,
                  const Pose &target,
                  const double maxVoltage) {
  Pose state{drive->getPose()};
  if(flipped) {
    state.flip();
  }
  const degree_t targetAngle{angle(state, target)};
  toward(timeout, targetAngle, maxVoltage);
}

void Turn::toward(const second_t timeout,
                  degree_t target,
                  const double maxVoltage) {
  interrupted = false;
  if(flipped) {
    target *= -1;
  }
  logger.debug("Turning to " + to_string(target) + ".");
  acceptable.reset(timeout);
  pid.reset();
  while(!acceptable.canAccept() && !interrupted) {
    const degree_t state{drive->getPose().h};
    double output{
        pid.getOutput(getValueAs<radian_t>(constrain180(target - state)))};
    output = std::clamp(output, -maxVoltage, maxVoltage);
    drive->arcade(0, output);
    acceptable.canAccept(state, target);
    wait();
  }
  drive->brake();
  if(interrupted) {
    logger.debug("Turn was interrupted!");
    interrupted = false;
  } else {
    logger.debug("Turn complete!");
  }
}

void Turn::awayFrom(const second_t timeout,
                    const Pose &target,
                    const double maxVoltage) {
  Pose state{drive->getPose()};
  if(flipped) {
    state.flip();
  }
  const degree_t targetAngle{angle(state, target)};
  awayFrom(timeout, targetAngle, maxVoltage);
}

void Turn::awayFrom(const second_t timeout,
                    const degree_t target,
                    const double maxVoltage) {
  toward(timeout, target + 180_deg, maxVoltage);
}
} // namespace atum