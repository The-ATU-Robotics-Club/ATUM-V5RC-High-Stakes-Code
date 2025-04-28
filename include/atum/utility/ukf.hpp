#pragma once

#include "eigen.hpp"
#include <iostream>

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
    xHat.setZero();
    chiF.setZero();
    xHatPriori.setZero();
    PPriori.setZero();
    yHatPriori.setZero();
    upsilon.setZero();

    const double alpha2{alpha * alpha};
    const double lambda{TotalStates * (alpha2 - 1.0)};
    Wm[0] = lambda / (TotalStates + lambda);
    Wc[0] = Wm[0] + (1.0 - alpha2 + beta);
    for(int i{0}; i < Wm.size(); i++) {
      Wm[i] = Wc[i] = 1.0 / (2.0 * (TotalStates + lambda));
    }
    eta = std::sqrt(TotalStates + lambda);
    std::cout << "ETA: " << eta << '\n';
  }

  virtual void initialize(const State &x) {
    xHat = x;
  }

  State getState() const {
    return xHat;
  }

  void predict() {
    const Input u{getInput()};
    std::cout << "LINE: " << __LINE__ << '\n' << u << '\n';

    std::cout << "LINE: " << __LINE__ << '\n' << P << '\n';
    Eigen::LLT<StateCovariance> chol{P};
    StateCovariance PSqrt{chol.matrixL()};
    std::cout << "LINE: " << __LINE__ << '\n' << PSqrt << '\n';
    SigmaPoints chi;
    chi << xHat, merweAdd(eta * PSqrt, xHat), merweAdd(-eta * PSqrt, xHat);
    std::cout << "LINE: " << __LINE__ << '\n' << chi << '\n';

    chiF = chi;
    for(auto point : chiF.colwise()) {
      point = f(point, u);
    }
    std::cout << "LINE: " << __LINE__ << '\n' << chiF << '\n';

    xHatPriori.setZero();
    for(int i{0}; i < Wm.size(); i++) {
      xHatPriori += Wm[i] * chiF.col(i);
    }
    std::cout << "LINE: " << __LINE__ << '\n' << xHatPriori << '\n';

    PPriori = Q;
    for(int i{0}; i < Wc.size(); i++) {
      const State diff{chiF.col(i) - xHatPriori};
      PPriori += Wc[i] * diff * diff.transpose();
    }
    std::cout << "LINE: " << __LINE__ << '\n' << PPriori << '\n';

    upsilon.setZero();
    for(int i{0}; i < chiF.cols(); i++) {
      upsilon.col(i) = h(chiF.col(i), u);
    }
    std::cout << "LINE: " << __LINE__ << '\n' << upsilon << '\n';

    yHatPriori.setZero();
    for(int i{0}; i < Wm.size(); i++) {
      yHatPriori += Wm[i] * upsilon.col(i);
    }
    std::cout << "LINE: " << __LINE__ << '\n' << yHatPriori << '\n';
  }

  void correct() {
    const Output y{getOutput()};
    std::cout << "LINE: " << __LINE__ << '\n' << y << '\n';

    OutputCovariance Py{R};
    for(int i{0}; i < Wc.size(); i++) {
      const Output diff{upsilon.col(i) - yHatPriori};
      Py += Wc[i] * diff * diff.transpose();
    }
    std::cout << "LINE: " << __LINE__ << '\n' << Py << '\n';

    Eigen::Matrix<double, TotalStates, TotalOutputs> Pxy;
    for(int i{0}; i < Wc.size(); i++) {
      const State xDiff{chiF.col(i) - xHatPriori};
      const Output yDiff{upsilon.col(i) - yHatPriori};
      Pxy += Wc[i] * xDiff * yDiff.transpose();
    }
    std::cout << "LINE: " << __LINE__ << '\n' << Pxy << '\n';

    const KalmanGain K{Pxy * Py.inverse()};
    std::cout << "LINE: " << __LINE__ << '\n' << K << '\n';

    xHat = xHatPriori + K * (y - yHatPriori);
    P = PPriori - K * Py * K.transpose();
    std::cout << "LINE: " << __LINE__ << '\n' << P << '\n';
  }

  protected:
  virtual State f(const State &x, const Input &u) = 0;

  virtual Output h(const State &x, const Input &u) = 0;

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
  StateCovariance PPriori;
  Output yHatPriori;
  SigmaOutputPoints upsilon;
};
} // namespace atum