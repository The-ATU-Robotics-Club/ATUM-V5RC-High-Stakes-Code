#pragma once

#include "atum/atum.hpp"

namespace atum {
using PoseEstimatorUKF = UKF<6, 2, 5>;

class PoseEstimator : public PoseEstimatorUKF {
  public:
  PoseEstimator() :
      PoseEstimatorUKF{PoseEstimator::StateCovariance{},
                       PoseEstimator::OutputCovariance{}} {}

  private:
  State f(State x, Input u) override {}

  Output h(State x, Input u) override {}

  Input getInput() override {}

  Output getOutput() override {}
};
} // namespace atum