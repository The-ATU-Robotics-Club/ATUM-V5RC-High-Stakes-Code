#pragma once

#include "../controllers/controller.hpp"
#include "../utility/acceptable.hpp"
#include "motionProfile.hpp"

namespace atum {
template <typename Unit>
class ProfileFollower {
  public:
  // Figure out the corresponding types for the derivative of Unit.
  using UnitsPerSecond = decltype(Unit{1} / 1_s);
  using UnitsPerSecondSq = decltype(UnitsPerSecond{1} / 1_s);
  using UnitProfile = MotionProfile<Unit>;
  using UnitAcceptable = Acceptable<Unit>;

  struct AccelerationConstants {
    double accel;
    double decel;
  };

  ProfileFollower(const UnitProfile &iProfile,
                  const UnitAcceptable &iAcceptable,
                  std::unique_ptr<Controller> iVelocityController,
                  const AccelerationConstants &iKA,
                  std::unique_ptr<Controller> iPositionController = nullptr,
                  const double iTimeoutScaling = 1.0,
                  const Logger::Level loggerLevel = Logger::Level::Info) :
      profile{iProfile},
      acceptable{iAcceptable},
      velocityController{std::move(iVelocityController)},
      kA{iKA},
      positionController{std::move(iPositionController)},
      timeoutScaling{iTimeoutScaling},
      logger{loggerLevel} {
    if(!velocityController) {
      logger.error("Must provide a velocity controller to profile follower!");
    }
    logger.debug("Profile follower constructed!");
  }

  void startProfile(const Unit &start, const Unit &iEnd) {
    end = iEnd;
    profile.generate(start, end);
    acceptable.reset(timeoutScaling * profile.getTotalTime());
    velocityController->reset();
    if(positionController) {
      positionController->reset();
    }
    logger.debug("Profile follower to follow profile from " + to_string(start) +
                 " to " + to_string(end) + ".");
    if(logger.getLevel() == Logger::Level::Debug) {
      prepareGraphing(start);
    }
  }

  double getOutput(const Unit &s, const UnitsPerSecond &v) {
    if(logger.getLevel() == Logger::Level::Debug) {
      graphPoint(s, v);
    }
    const typename UnitProfile::Point reference{profile.getPoint(s)};
    const double velocityOutput{
        velocityController->getOutput(getValueAs<UnitsPerSecond>(v),
                                      getValueAs<UnitsPerSecond>(reference.v))};
    double accelerationOutput{getValueAs<UnitsPerSecondSq>(reference.a)};
    if(reference.a < UnitsPerSecondSq{0}) {
      accelerationOutput *= profile.isBackwards() ? kA.accel : kA.decel;
    } else {
      accelerationOutput *= profile.isBackwards() ? kA.decel : kA.accel;
    }
    double positionOutput{0.0};
    if(positionController) {
      const double positionError{difference(reference.s, s)};
      positionOutput = positionController->getOutput(positionError);
    }
    acceptable.canAccept(s, end);
    return positionOutput + velocityOutput + accelerationOutput;
  }

  bool isDone() {
    return acceptable.canAccept();
  }

  private:
  void prepareGraphing(const Unit start) {
    const typename UnitProfile::Parameters motionParams{
        profile.getParameters()};
    GUI::Graph::clearSeries(GUI::SeriesColor::Green);
    GUI::Graph::clearSeries(GUI::SeriesColor::Cyan);
    const double rawStart{getValueAs<Unit>(start)};
    const double rawEnd{getValueAs<Unit>(end)};
    GUI::Graph::setSeriesRange(
        {std::min(rawStart, rawEnd), std::max(rawStart, rawEnd)},
        GUI::SeriesColor::Green);
    GUI::Graph::setSeriesRange(getValueAs<UnitsPerSecond>(motionParams.maxV),
                               GUI::SeriesColor::Cyan);
  }

  void graphPoint(const Unit s, const UnitsPerSecond v) {
    GUI::Graph::addValue(getValueAs<Unit>(s), GUI::SeriesColor::Green);
    GUI::Graph::addValue(getValueAs<UnitsPerSecond>(v), GUI::SeriesColor::Cyan);
  }

  UnitProfile profile;
  UnitAcceptable acceptable;
  std::unique_ptr<Controller> velocityController;
  AccelerationConstants kA;
  std::unique_ptr<Controller> positionController;
  const double timeoutScaling;
  Logger logger;
  Unit end;
};

using LateralProfileFollower = ProfileFollower<meter_t>;
using AngularProfileFollower = ProfileFollower<radian_t>;
} // namespace atum