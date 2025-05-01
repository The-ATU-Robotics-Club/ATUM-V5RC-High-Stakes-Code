#pragma once

#include "atum/atum.hpp"


namespace atum {
class PoseEstimator : public EKF<6, 2, 8>, public Tracker, public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  PoseEstimator(Drive *iDrive,
                std::unique_ptr<Odometry> iOdometry,
                std::unique_ptr<OTOS> iOTOS,
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

  void setPoseInternal(const Pose &iPose);

  Drive *drive;
  std::unique_ptr<Odometry> odometry;
  std::unique_ptr<OTOS> otos;

  double D1, D2, D3, D4;
};
} // namespace atum