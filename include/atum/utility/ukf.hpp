#pragma once

#include "../depend/Eigen/Eigen"


namespace atum {
template <int TotalStates, int TotalInputs, int TotalOutputs>
class UKF {
  public:
  using State = Eigen::Vector<double, TotalStates>;
  using StateCovariance = Eigen::Matrix<double, TotalStates, TotalStates>;
  using Input = Eigen::Vector<double, TotalInputs>;
  using KalmanGain = Eigen::Matrix<double, TotalStates, TotalOutputs>;
  using OutputVector = Eigen::Vector<double, TotalOutputs>;
  using OutputCovariance = Eigen::Matrix<double, TotalOutputs, TotalOutputs>;

  UKF(const StateCovariance &iP,
      const StateCovariance &iQ,
      const OutputCovariance &iR,
      const State &iX = State{}) :
      P{iP},
      Q{iQ},
      R{iR},
      x{iX} {}

  private:
  State x;
  StateCovariance P;
  const StateCovariance Q;
  const OutputCovariance R;
};
} // namespace atum