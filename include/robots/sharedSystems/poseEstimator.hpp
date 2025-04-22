#pragma once

#include "atum/atum.hpp"


namespace atum {
class PoseEstimator : public UKF<6, 2, 5> {
  public:
  PoseEstimator();

  private:
  State f(State x, Input u) override;

  Output h(State x, Input u) override;

  Input getInput() override;

  Output getOutput() override;
};
} // namespace atum