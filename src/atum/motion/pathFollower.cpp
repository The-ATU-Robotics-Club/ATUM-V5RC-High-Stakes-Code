#include "pathFollower.hpp"

namespace atum {
PathFollower::Command::Command(const Pose &iTarget,
                               bool iReversed,
                               std::optional<Path::Parameters> iParams,
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
                           const AccelerationConstants &iKA,
                           const FeedbackParams &iFeedbackParams,
                           const Logger::Level loggerLevel) :
    drive{iDrive},
    defaultAcceptable{iDefaultAcceptable},
    left{std::move(iLeft)},
    right{std::move(iRight)},
    kA{iKA},
    feedbackParams{iFeedbackParams},
    logger{loggerLevel} {
  prepareGraph();
}

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
  Path traj{{state, cmd.target}, cmd.params, logger.getLevel()};
  while(!acceptable.canAccept(distance(drive->getPose(), cmd.target)) &&
        !interrupted) {
    UnwrappedPose state{drive->getPose()};
    if(cmd.reversed) {
      state.h += M_PI;
    }
    const UnwrappedPose target{traj.getPose(drive->getPose())};
    const auto [refV, refOmega] = getReference(state, target);
    const auto [refVL, refVR] = toRPM(refV, refOmega);
    const auto [vL, vR] = drive->getLRVelocity();
    const double stateVL{getValueAs<revolutions_per_minute_t>(vL)};
    const double stateVR{getValueAs<revolutions_per_minute_t>(vR)};
    const auto [ffL, ffR] = getAccelFeedforward(refV, refOmega);
    if(cmd.reversed) {
      drive->tank(left->getOutput(stateVL, -refVR) - ffR,
                  right->getOutput(stateVR, -refVL) - ffL);
    } else {
      drive->tank(left->getOutput(stateVL, refVL) + ffL,
                  right->getOutput(stateVR, refVR) + ffR);
    }
    graphPoints(stateVL, refVL, stateVR, refVR);
    wait(5_ms);
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

std::pair<double, double>
    PathFollower::getReference(const UnwrappedPose &state,
                               const UnwrappedPose &target) {
  if(!feedbackParams.useRAMSETE) {
    return {target.v, target.omega};
  }
  const UnwrappedPose error{getError(state, target)};
  const double k{
      2.0 * feedbackParams.lambda *
      sqrt(pow(target.omega, 2.0) + feedbackParams.beta * pow(target.v, 2.0))};
  const double v{target.v * cos(error.h) + k * error.x};
  const double sincHError{error.h ? sin(error.h) / error.h : 1.0};
  const double omega{target.omega + k * error.h +
                     feedbackParams.beta * target.v * sincHError * error.y};
  return {v, omega};
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

std::pair<double, double> PathFollower::toRPM(const double v,
                                              const double omega) {
  auto [leftV, rightV] = toLR(v, omega);
  const double maxV{getValueAs<meters_per_second_t>(drive->getMaxVelocity())};
  if(abs(leftV) > maxV) {
    rightV -= leftV - maxV;
    leftV = leftV > 0 ? maxV : -maxV;
  } else if(abs(rightV) > maxV) {
    leftV -= rightV - maxV;
    rightV = rightV > 0 ? maxV : -maxV;
  }
  const double mpsToRPM{60.0 / drive->getGeometry().circum};
  return {mpsToRPM * leftV, mpsToRPM * rightV};
}

std::pair<double, double>
    PathFollower::getAccelFeedforward(const double a, const double alpha) {
  auto [ffL, ffR] = toLR(a, alpha);
  const double multiplier{a > 0.0 ? kA.accel : kA.decel};
  return {ffL, ffR};
}

std::pair<double, double> PathFollower::toLR(const double lateral,
                                             const double angular) {
  const Drive::Geometry geometry{drive->getGeometry()};
  const double track{getValueAs<meter_t>(geometry.track)};
  const double angularAdjustment{angular * track / 2.0};
  double leftV{lateral + angularAdjustment};
  double rightV{lateral - angularAdjustment};
  return {leftV, rightV};
}

void PathFollower::prepareGraph() {
  if(logger.getLevel() != Logger::Level::Debug) {
    return;
  }
  GUI::Graph::clearSeries(GUI::SeriesColor::Magenta);
  GUI::Graph::clearSeries(GUI::SeriesColor::Cyan);
  const double maxV{getValueAs<revolutions_per_minute_t>(drive->getMaxRPM())};
  GUI::Graph::setSeriesRange(maxV, GUI::SeriesColor::Red);
  GUI::Graph::setSeriesRange(maxV, GUI::SeriesColor::Magenta);
  GUI::Graph::setSeriesRange(maxV, GUI::SeriesColor::Blue);
  GUI::Graph::setSeriesRange(maxV, GUI::SeriesColor::Cyan);
}

void PathFollower::graphPoints(const double stateVL,
                               const double refVL,
                               const double stateVR,
                               const double refVR) {
  if(logger.getLevel() != Logger::Level::Debug) {
    return;
  }
  GUI::Graph::addValue(stateVL, GUI::SeriesColor::Red);
  GUI::Graph::addValue(refVL, GUI::SeriesColor::Magenta);
  GUI::Graph::addValue(stateVR, GUI::SeriesColor::Blue);
  GUI::Graph::addValue(refVR, GUI::SeriesColor::Cyan);
}
} // namespace atum