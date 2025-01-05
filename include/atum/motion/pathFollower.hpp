#pragma once

#include "../controllers/controller.hpp"
#include "../systems/drive.hpp"
#include "../utility/acceptable.hpp"
#include "path.hpp"
#include "profileFollower.hpp"

namespace atum {
class PathFollower {
  public:
  struct Command {
    Command(const Pose &iTarget,
            bool iReversed = false,
            std::optional<Path::Parameters> iParams = {},
            std::optional<AcceptableDistance> iAcceptable = {});
    Pose target;
    bool reversed{false};
    std::optional<Path::Parameters> params;
    std::optional<AcceptableDistance> acceptable;
  };

  struct FeedbackParams {
    bool useRAMSETE{true};
    double beta{2.0};
    double lambda{0.7};
  };

  PathFollower(Drive *iDrive,
               const AcceptableDistance &iDefaultAcceptable,
               std::unique_ptr<Controller> iLeft,
               std::unique_ptr<Controller> iRight,
               const AccelerationConstants &iKA,
               const FeedbackParams &iFeedbackParams = FeedbackParams{true,
                                                                      2.0,
                                                                      0.7},
               const Logger::Level loggerLevel = Logger::Level::Info);

  void follow(const std::vector<Command> &commands);

  void interrupt();

  private:
  void follow(Command cmd);

  std::pair<double, double> getReference(const UnwrappedPose &state,
                                         const UnwrappedPose &target);

  UnwrappedPose getError(const UnwrappedPose &state,
                         const UnwrappedPose &target);

  std::pair<double, double> toRPM(const double v, const double omega);

  double getAccelFeedforward(const double a, const bool reversed);

  std::pair<double, double> toLR(const double lateral, const double angular);

  void prepareGraph();

  void graphPoints(const double stateVL,
                   const double refVL,
                   const double stateVR,
                   const double refVR);

  Drive *drive;
  AcceptableDistance defaultAcceptable;
  std::unique_ptr<Controller> left;
  std::unique_ptr<Controller> right;
  AccelerationConstants kA;
  FeedbackParams feedbackParams;
  Logger logger;
  bool interrupted{false};
};
} // namespace atum