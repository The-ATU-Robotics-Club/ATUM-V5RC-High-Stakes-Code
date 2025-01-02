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
    const scalar_t sinH{sin(target.h)};
    const scalar_t cosH{cos(target.h)};
    target.h = atan2(sinH, -cosH);
  }
}

PathFollower::PathFollower(Drive *iDrive,
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

void PathFollower::follow(const std::vector<Command> &commands) {
  for(int i{0}; i < commands.size() && !interrupted; i++) {
    follow(commands[i]);
  }
}

void PathFollower::follow(Command cmd) {
  Acceptable acceptable{cmd.acceptable.value_or(defaultAcceptable)};
  acceptable.reset();
  left->reset();
  right->reset();
  Pose state{drive->getPose()};
  if(cmd.reversed) {
    state.h += 180_deg;
    cmd.target.h += 180_deg;
  }
  Trajectory traj{{state, cmd.target}, cmd.params, logger.getLevel()};
  std::cout << __LINE__ << '\n';
  while(acceptable.canAccept(distance(drive->getPose(), cmd.target)) &&
        !interrupted) {
  std::cout << __LINE__ << '\n';
    UnwrappedPose state{drive->getPose()};
  std::cout << __LINE__ << '\n';
    if(cmd.reversed) {
      state.h += M_PI;
    }
  std::cout << __LINE__ << '\n';
    const UnwrappedPose target{traj.getPose(drive->getPose())};
  std::cout << __LINE__ << '\n';
    const UnwrappedPose error{getError(state, target)};
    const double k{2.0 * lambda *
                   sqrt(pow(target.w, 2.0) + beta * pow(target.v, 2.0))};
    const double v{target.v * cos(error.h) + k * error.x};
    const double sincHError{error.h ? sin(error.h) / error.h : 1.0};
    const double w{target.w + k * error.h +
                   beta * target.v * sincHError * error.y};
    const auto [refVL, refVR] = toLRVelocity(v, w);
    const auto [vL, vR] = drive->getLRVelocity();
    logger.debug(std::to_string(refVL));
    const double stateVL{getValueAs<revolutions_per_minute_t>(vL)};
    const double stateVR{getValueAs<revolutions_per_minute_t>(vR)};
    if(cmd.reversed)
      drive->tank(left->getOutput(stateVL, -refVR),
                  right->getOutput(stateVR, -refVL));
    else
      drive->tank(left->getOutput(stateVL, refVL),
                  right->getOutput(stateVR, refVR));
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

std::pair<double, double> PathFollower::toLRVelocity(const double v,
                                                     const double w) {
  const Drive::Geometry geometry{drive->getGeometry()};
  const double track{getValueAs<meter_t>(geometry.track)};
  const double circum{getValueAs<meter_t>(geometry.circum)};
  const double angularAdjustment{w * track / 2.0};
  return std::make_pair((v + angularAdjustment) / circum,
                        (v - angularAdjustment) / circum);
}

} // namespace atum