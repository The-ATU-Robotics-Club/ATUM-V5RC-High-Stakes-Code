#pragma once

#include "eigen.hpp"
#include <iostream>

namespace atum {
template <int TotalStates, int TotalInputs, int TotalOutputs>
class EKF {
  public:
  using State = Eigen::Vector<double, TotalStates>;
  using Input = Eigen::Vector<double, TotalInputs>;
  using Output = Eigen::Vector<double, TotalOutputs>;
  using StateCovariance = Eigen::Matrix<double, TotalStates, TotalStates>;
  using OutputCovariance = Eigen::Matrix<double, TotalOutputs, TotalOutputs>;
  using KalmanGain = Eigen::Matrix<double, TotalStates, TotalOutputs>;
  using JacobianF = Eigen::Matrix<double, TotalStates, TotalStates>;
  using JacobianH = Eigen::Matrix<double, TotalOutputs, TotalStates>;

  EKF(const StateCovariance &iP,
      const StateCovariance &iQ,
      const OutputCovariance &iR) :
      P{iP},
      Q{iQ},
      R{iR} {
    xHat.setZero();
    xHatPriori.setZero();
    PPriori.setZero();
  }

  virtual void initialize(const State &x) {
    xHat = x;
  }

  State getState() const {
    return xHat;
  }

  void predict() {
    const Input u{getInput()};
    xHatPriori = f(xHat, u);
    const JacobianF F{linearF(xHat, u)};
    PPriori = F * P * F.transpose() + Q;
  }

  void correct() {
    const Output z{getOutput()};
    const Output y{z - h(xHatPriori)};
    const JacobianH H{linearH(xHatPriori)};
    OutputCovariance S{H * PPriori * H.transpose() + R};
    const KalmanGain K{PPriori * H.transpose() * S.inverse()};
    xHat = xHatPriori + K * y;
    P = (StateCovariance::Identity() - K * H) * PPriori;
  }

  protected:
  virtual State f(const State &x, const Input &u) = 0;

  virtual Output h(const State &x) = 0;

  virtual JacobianF linearF(const State &x, const Input &u) = 0;

  virtual JacobianH linearH(const State &x) = 0;

  virtual Input getInput() = 0;

  virtual Output getOutput() = 0;

  State xHat;
  StateCovariance P;
  const StateCovariance Q;
  const OutputCovariance R;
  State xHatPriori;
  StateCovariance PPriori;
};
} // namespace atum