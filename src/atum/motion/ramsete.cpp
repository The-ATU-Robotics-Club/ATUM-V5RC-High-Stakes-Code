#include "ramsete.hpp"

namespace atum {
RAMSETE::RAMSETE(Drive *iDrive,
                 const AcceptableDistance &iDefaultAcceptable,
                 std::unique_ptr<Controller> iLeft,
                 std::unique_ptr<Controller> iRight,
                 const double iBeta,
                 const double iLambda,
                 const Logger::Level loggerLevel) :
    drive{iDrive},
    defaultAcceptable{iDefaultAcceptable},
    left{std::move(iLeft)},
    right{std::move(iRight)},
    beta{iBeta},
    lambda{iLambda},
    logger{loggerLevel} {}

void RAMSETE::follow(const std::vector<Command> &commands) {
  for(int i{0}; i < commands.size() && !interrupted; i++) {
    follow(commands[i]);
  }
}

void RAMSETE::follow(Command cmd) {
  left->reset();
  right->reset();
  Acceptable acceptable{cmd.acceptable.value_or(defaultAcceptable)};
  Pose state{drive->getPose()};
  if(cmd.reversed) {
    state.h += 180_deg;
    cmd.target.h += 180_deg;
  }
  Trajectory traj{{state, cmd.target}, cmd.params, logger.getLevel()};
  while(acceptable.canAccept(distance(drive->getPose(), cmd.target)) ||
        !interrupted) {
    UnwrappedPose state{drive->getPose()};
    if(cmd.reversed) {
      state.h += M_PI;
    }
    const UnwrappedPose target{cmd.target};
    const UnwrappedPose error{getError(state, cmd.target)};
    const double k{2.0 * lambda *
                   sqrt(pow(target.w, 2.0) + beta * pow(target.v, 2.0))};
    const double v{target.v * cos(error.h) + k * error.x};
    const double sincHError{error.h ? sin(error.h) / error.h : 1.0};
    const double w{target.w + k * error.h +
                   beta * target.v * sincHError * error.y};
    const auto [refVL, refVR] = drive->toLRVelocity(v, w);
    const auto [stateVL, stateVR] = drive->getLRVelocity();
    if(cmd.reversed) {
      drive->tank(left->getOutput(stateVL, -refVR),
                  right->getOutput(stateVR, -refVL));
    } else {
      drive->tank(left->getOutput(stateVL, refVL),
                  right->getOutput(stateVR, refVR));
    }
    wait();
  }
  drive->tank(0, 0);
  interrupted = false;
}

void RAMSETE::interrupt() {
  interrupted = true;
}

UnwrappedPose RAMSETE::getError(const UnwrappedPose &state,
                                const UnwrappedPose &target) {
  UnwrappedPose error{target - state};
  const double globalXError{error.x * sin(state.h) + error.y * cos(state.h)};
  const double globalYError{error.x * cos(state.h) - error.y * sin(state.h)};
  error.x = globalXError;
  error.y = globalYError;
  error.h = constrainPI(target.h - state.h);
  return error;
}

} // namespace atum