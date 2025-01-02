#pragma once

#include "../controllers/controller.hpp"
#include "../systems/drive.hpp"
#include "../utility/acceptable.hpp"
#include "trajectory.hpp"

namespace atum {
class PathFollower {
  public:
  struct Command {
    Command(const Pose &iTarget,
            bool iReversed = false,
            std::optional<Trajectory::Parameters> iParams = {},
            std::optional<AcceptableDistance> iAcceptable = {});
    Pose target;
    bool reversed{false};
    std::optional<Trajectory::Parameters> params;
    std::optional<AcceptableDistance> acceptable;
  };

  PathFollower(Drive *iDrive,
               const AcceptableDistance &iDefaultAcceptable,
               std::unique_ptr<Controller> iForward,
               std::unique_ptr<Controller> iTurn,
               const double iBeta = 2.0,
               const double iLambda = 0.7,
               const Logger::Level loggerLevel = Logger::Level::Info);

  void follow(const std::vector<Command> &commands);

  void follow(Command cmd);

  void interrupt();

  private:
  UnwrappedPose getError(const UnwrappedPose &state,
                         const UnwrappedPose &target);

  Drive *drive;
  AcceptableDistance defaultAcceptable;
  std::unique_ptr<Controller> forward;
  std::unique_ptr<Controller> turn;
  const double beta;
  const double lambda;
  Logger logger;
  bool interrupted{false};
};
} // namespace atum