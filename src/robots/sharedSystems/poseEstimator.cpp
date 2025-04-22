#include "poseEstimator.hpp"


namespace atum {

PoseEstimator::PoseEstimator() :
    UKF<6, 2, 5>{PoseEstimator::StateCovariance{},
                 PoseEstimator::OutputCovariance{}} {}

PoseEstimator::State PoseEstimator::f(PoseEstimator::State x,
                                      PoseEstimator::Input u) {
  return State{};
}

PoseEstimator::Output PoseEstimator::h(PoseEstimator::State x,
                                       PoseEstimator::Input u) {
  return Output{};
}

PoseEstimator::Input PoseEstimator::getInput() {
  return Input{};
}

PoseEstimator::Output PoseEstimator::getOutput() {
  return Output{};
}
} // namespace atum