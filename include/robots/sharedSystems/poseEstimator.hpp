#pragma once

#include "atum/atum.hpp"

namespace atum {
class PoseEstimator : public EKF<6, 2, 2>, public Tracker, public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  PoseEstimator(Drive *iDrive,
                const PoseEstimator::StateCovariance &iP,
                const PoseEstimator::StateCovariance &iQ,
                const PoseEstimator::OutputCovariance &iR);

  Pose update() override;

  void setPose(const Pose &iPose) override;

  private:
  static constexpr double dt{0.01};

  State f(const State &x, const Input &u) override;

  Output h(const State &x) override;

  JacobianF linearF(const State &x, const Input &u) override;

  JacobianH linearH(const State &x) override;

  Input getInput() override;

  Output getOutput() override;

  Drive *drive;

  double D1, D2, D3, D4;
};
} // namespace atum