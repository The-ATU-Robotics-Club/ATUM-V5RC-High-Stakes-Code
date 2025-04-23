#pragma once

#include "eigen.hpp"

namespace atum {
template <int TotalStates, int TotalInputs, int TotalOutputs>
class UKF {
  public:
  using State = Eigen::Vector<double, TotalStates>;
  using StateCovariance = Eigen::Matrix<double, TotalStates, TotalStates>;
  using Input = Eigen::Vector<double, TotalInputs>;
  using KalmanGain = Eigen::Matrix<double, TotalStates, TotalOutputs>;
  using Output = Eigen::Vector<double, TotalOutputs>;
  using OutputCovariance = Eigen::Matrix<double, TotalOutputs, TotalOutputs>;
  using SigmaPoints = Eigen::Matrix<double, TotalStates, 2 * TotalStates + 1>;
  using SigmaOutputPoints =
      Eigen::Matrix<double, TotalOutputs, 2 * TotalStates + 1>;
  using Weights = std::array<double, 2 * TotalStates + 1>;

  UKF(const StateCovariance &iP,
      const StateCovariance &iQ,
      const OutputCovariance &iR,
      const double alpha = 0.5,
      const double beta = 2.0) :
      P{iP},
      Q{iQ},
      R{iR} {
    const double alpha2{alpha * alpha};
    const double lambda{TotalStates * (alpha2 - 1.0)};
    Wm[0] = lambda / (TotalStates + lambda);
    Wc[0] = Wm[0] + (1.0 - alpha2 + beta);
    for(int i{0}; i < Wm.size(); i++) {
      Wm[i] = Wc[i] = 1.0 / (2.0 * (TotalStates + lambda));
    }
    eta = std::sqrt(TotalStates + lambda);
  }

  virtual void initialize(const State &x) {
    xHat = x;
  }

  State getState() const {
    return xHat;
  }

  void predict() {
    const Input u{getInput()};

    StateCovariance PSqrt{P.sqrt()};
    SigmaPoints chi;
    chi << xHat, merweAdd(eta * PSqrt, xHat), merweAdd(-eta * PSqrt, xHat);

    chiF = chi;
    for(auto point : chiF.colwise()) {
      point = f(point, u);
    }

    xHatPriori = State{};
    for(int i{0}; i < chiF.size(); i++) {
      xHatPriori += Wm[i] * chiF.col(i);
    }

    PPriori = Q;
    for(int i{0}; i < Wc.size(); i++) {
      const State diff{chiF.col(i) - xHatPriori};
      PPriori += Wc[i] * diff * diff.transpose();
    }

    upsilon = SigmaOutputPoints{};
    for(int i{0}; i < chiF.cols(); i++) {
      upsilon.col(i) = h(chiF.col(i), u);
    }

    yHatPriori = Output{};
    for(int i{0}; i < upsilon.size(); i++) {
      yHatPriori += Wm[i] * upsilon.col(i);
    }
  }

  void update() {
    const Output y{getOutput()};

    OutputCovariance Py{R};
    for(int i{0}; i < Wc.size(); i++) {
      const Output diff{upsilon.col(i) - yHatPriori};
      Py += Wc[i] * diff * diff.transpose();
    }

    Eigen::Matrix<double, TotalStates, TotalOutputs> Pxy;
    for(int i{0}; i < Wc.size(); i++) {
      const State xDiff{chiF.col(i) - xHatPriori};
      const Output yDiff{upsilon.col(i) - yHatPriori};
      Pxy += Wc[i] * xDiff * yDiff.transpose();
    }

    const KalmanGain K{Pxy * Py.inverse()};

    xHat = xHatPriori + K * (y - yHatPriori);
    P = PPriori - K * Py * K.transpose();
  }

  private:
  virtual State f(State x, Input u) = 0;

  virtual Output h(State x, Input u) = 0;

  virtual Input getInput() = 0;

  virtual Output getOutput() = 0;

  Eigen::MatrixXd merweAdd(Eigen::MatrixXd A, const Eigen::VectorXd &u) {
    for(auto col : A.colwise()) {
      col += u;
    }
    return A;
  }

  State xHat;
  StateCovariance P;
  const StateCovariance Q;
  const OutputCovariance R;
  Weights Wm, Wc;
  double eta;
  SigmaPoints chiF;
  State xHatPriori;
  StateCovariance PPriori{Q};
  Output yHatPriori;
  SigmaOutputPoints upsilon;
};
} // namespace atum