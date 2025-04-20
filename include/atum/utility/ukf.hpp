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
  using Weights = std::array<double, 2 * TotalStates + 1>;

  UKF(const StateCovariance &iQ,
      const OutputCovariance &iR,
      const double alpha = 0.5,
      const double beta = 2.0) :
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
    predict();
  }

  virtual void initialize(const State &x, const StateCovariance &P) {
    xHat = x;
    // TODO: see about llt method instead of this.
    S = Eigen::LLT<StateCovariance>{P};
  }

  State getState() const {
    return xHat;
  }

  void predict() {
    const Input u{getInput()};

    SigmaPoints chi;
    StateCovariance SFactor{S.matrixL()};
    chi << xHat, merweAdd(eta * SFactor, xHat), merweAdd(-eta * SFactor, xHat);

    SigmaPoints chiPropagated{chi};
    for(auto point : chiPropagated.colwise()) {
      point = f(point, u);
    }

    State xHatPriori;
    for(int i{0}; i < chiPropagated.size(); i++) {
      xHatPriori += Wm[i] * chiPropagated.col(i);
    }

    StateCovariance SPrioriTemp0{
        std::sqrt(Wc[1]) *
        merweAdd(chi.rightCols(chi.cols() - 1), -xHatPriori)};
    SPrioriTemp0 << Q.sqrt();
    StateCovariance SPrioriTemp1{
        Eigen::HouseholderQR<StateCovariance>{SPrioriTemp0}
            .matrixQR()
            .template triangularView<Eigen::Upper>()};

    Eigen::LLT<StateCovariance> SPriori = SPrioriTemp1;
    SPriori.rankUpdate(chi.col(0) - xHatPriori);

    SigmaPoints chiOutput{chi};
    for(auto point : chiPropagated.colwise()) {
      point = h(point);
    }

    State yHatPriori;
    for(int i{0}; i < chiPropagated.size(); i++) {
      yHatPriori += Wm[i] * chiPropagated.col(i);
    }
  }

  void update() {}

  private:
  virtual State f(State x, Input u) = 0;

  virtual Input h(State x, Input u) = 0;

  virtual Input getInput() = 0;

  virtual Output getOutput() = 0;

  Eigen::MatrixXd merweAdd(Eigen::MatrixXd A, const Eigen::VectorXd &u) {
    for(auto col : A.colwise()) {
      col += u;
    }
    return A;
  }

  State xHat;
  Eigen::LLT<StateCovariance> S;
  Weights Wm, Wc;
  double eta;
  const StateCovariance Q;
  const OutputCovariance R;
};
} // namespace atum