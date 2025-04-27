#pragma once

#include "atum/atum.hpp"


namespace atum {
class PoseEstimator : public UKF<6, 2, 2>, public Tracker, public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  PoseEstimator(Drive *iDrive,
                const PoseEstimator::StateCovariance &iP,
                const PoseEstimator::StateCovariance &iQ,
                const PoseEstimator::OutputCovariance &iR,
                const double alpha = 0.9,
                const double beta = 2.0);

  Pose update() override;

  void setPose(const Pose &iPose) override;

  Pose getPose() override;

  private:
  static constexpr double dt{0.01};

  State f(const State &x, const Input &u) override;

  Output h(const State &x, const Input &u) override;

  Input getInput() override;

  Output getOutput() override;

  Drive *drive;
};
} // namespace atum