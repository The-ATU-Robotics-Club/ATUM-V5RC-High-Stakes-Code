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

  UKF(const StateCovariance &Q,
      const OutputCovariance &R,
      const double alpha = 0.5,
      const double beta = 2.0) :
      QSqrt{Q.sqrt()}, RSqrt{R.sqrt()} {
    const double alpha2{alpha * alpha};
    const double lambda{TotalStates * (alpha2 - 1.0)};
    Wm[0] = lambda / (TotalStates + lambda);
    Wc[0] = Wm[0] + (1.0 - alpha2 + beta);
    for(int i{0}; i < Wm.size(); i++) {
      Wm[i] = Wc[i] = 1.0 / (2.0 * (TotalStates + lambda));
    }
    eta = std::sqrt(TotalStates + lambda);
    predict();
  }

  virtual void initialize(const State &x, const StateCovariance &P) {
    xHat = x;
    // TODO: see about llt method instead of this.
    S = Eigen::LLT<StateCovariance>{P}.matrixL();
  }

  State getState() const {
    return xHat;
  }

  void predict() {
    // PREDICT STEP
    const Input u{getInput()};

    SigmaPoints chi;
    chi << xHat, merweAdd(eta * S, xHat), merweAdd(-eta * S, xHat);

    SigmaPoints chiPropagated{chi};
    for(auto point : chiPropagated.colwise()) {
      point = f(point, u);
    }

    State xHatPriori;
    for(int i{0}; i < chiPropagated.size(); i++) {
      xHatPriori += Wm[i] * chiPropagated.col(i);
    }

    StateCovariance SPriori{
        std::sqrt(Wc[1]) *
        merweAdd(chi.rightCols(chi.cols() - 1), -xHatPriori)};
    SPriori << QSqrt;
    SPriori = Eigen::HouseholderQR<StateCovariance>{SPriori}
                  .matrixQR()
                  .template triangularView<Eigen::Upper>();
    Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(
        SPriori, chi.col(0) - xHatPriori, Wc[0]);
    SPriori.transposeInPlace();

    SigmaOutputPoints chiOutput;
    for(int i{0}; i < chiPropagated.cols(); i++) {
      chiOutput.col(i) = h(chiPropagated.col(i), u);
    }

    Output yHatPriori;
    for(int i{0}; i < chiOutput.size(); i++) {
      yHatPriori += Wm[i] * chiOutput.col(i);
    }

    // UPDATE STEP
    OutputCovariance Sy{
        std::sqrt(Wc[1]) *
        merweAdd(chiOutput.rightCols(chiOutput.cols() - 1), -yHatPriori)};
    Sy << RSqrt;
    Sy = Eigen::HouseholderQR<OutputCovariance>{Sy}
             .matrixQR()
             .template triangularView<Eigen::Upper>();
    Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(
        Sy, chiOutput.col(0) - yHatPriori, Wc[0]);
    Sy.transposeInPlace();

    KalmanGain Pxy;
    for(int i{0}; i < Wc.size(); i++) {
      Pxy += Wc[i] * (chiPropagated.col(i) - xHatPriori) *
             (chiOutput.col(i) - yHatPriori).transpose();
    }

    KalmanGain K{(Pxy * Sy.transpose().inverse()) * Sy.inverse()};

    const Output y{getOutput()};
    xHat = xHatPriori + K * (y - yHatPriori);

    KalmanGain U{K * Sy};
    for(int i{0}; i < U.cols(); i++) {
      Eigen::internal::llt_inplace<double, Eigen::Lower>::rankUpdate(
          SPriori, U.col(i), -1.0);
    }
    S = SPriori;
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
  StateCovariance S;
  Weights Wm, Wc;
  double eta;
  const StateCovariance QSqrt;
  const OutputCovariance RSqrt;
};
} // namespace atum