#include "pathFollower.hpp"

namespace atum {
PathFollower::Command::Command(const Pose &iTarget,
                               bool iReversed,
                               std::optional<Trajectory::Parameters> iParams,
                               std::optional<AcceptableDistance> iAcceptable) :
    target{iTarget},
    reversed{iReversed},
    params{iParams},
    acceptable{iAcceptable} {
  if(GUI::Routines::selectedColor() == MatchColor::Blue) {
    target.x *= -1;
    target.h += 180_deg;
  }
}

PathFollower::PathFollower(Drive *iDrive,
                           const AcceptableDistance &iDefaultAcceptable,
                           std::unique_ptr<Controller> iForward,
                           std::unique_ptr<Controller> iTurn,
                           const double iBeta,
                           const double iLambda,
                           const Logger::Level loggerLevel) :
    drive{iDrive},
    defaultAcceptable{iDefaultAcceptable},
    forward{std::move(iForward)},
    turn{std::move(iTurn)},
    beta{iBeta},
    lambda{iLambda},
    logger{loggerLevel} {}

void PathFollower::follow(const std::vector<Command> &commands) {
  for(int i{0}; i < commands.size() && !interrupted; i++) {
    follow(commands[i]);
  }
}

void PathFollower::follow(Command cmd) {
  forward->reset();
  turn->reset();
  Acceptable acceptable{cmd.acceptable.value_or(defaultAcceptable)};
  Pose state{drive->getPose()};
  if(cmd.reversed) {
    state.h += 180_deg;
    cmd.target.h += 180_deg;
  }
  Trajectory traj{{state, cmd.target}, cmd.params, logger.getLevel()};
  while(acceptable.canAccept(distance(drive->getPose(), cmd.target)) &&
        !interrupted) {
    UnwrappedPose state{drive->getPose()};
    if(cmd.reversed) {
      state.h += M_PI;
    }
    const UnwrappedPose target{cmd.target}; // Change this
    const UnwrappedPose error{getError(state, target)};
    const double k{2.0 * lambda *
                   sqrt(pow(target.w, 2.0) + beta * pow(target.v, 2.0))};
    const double v{target.v * cos(error.h) + k * error.x};
    const double sincHError{error.h ? sin(error.h) / error.h : 1.0};
    const double w{target.w + k * error.h +
                   beta * target.v * sincHError * error.y};
    if(cmd.reversed) {
      drive->arcade(forward->getOutput(state.v, -v),
                    turn->getOutput(state.w, -w));
    } else {
      drive->arcade(forward->getOutput(state.v, v),
                    turn->getOutput(state.w, w));
    }
    wait();
  }
  drive->tank(0, 0);
  if(interrupted) {
    logger.debug("Path following was interrupted!");
    interrupted = false;
  }
}

void PathFollower::interrupt() {
  interrupted = true;
}

UnwrappedPose PathFollower::getError(const UnwrappedPose &state,
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