#include "atum/depend/units.h"
#include "atum/pose/pose.hpp"
#include "poseEstimator.hpp"


namespace atum {
PoseEstimator::PoseEstimator(Drive *iDrive,
                             const PoseEstimator::StateCovariance &iP,
                             const PoseEstimator::StateCovariance &iQ,
                             const PoseEstimator::OutputCovariance &iR,
                             const double alpha,
                             const double beta) :
    UKF{iP, iQ, iR, alpha, beta},
    Tracker{Logger::Level::Debug},
    Task(this, Logger::Level::Info),
    drive{iDrive} {}

Pose PoseEstimator::update() {
  const State state{getState()};
  const UnwrappedPose raw{
      state(0), state(1), state(2), state(3), state(4), state(5)};
  setPose(raw);
  return Tracker::getPose();
}

void PoseEstimator::setPose(const Pose &iPose) {
  const UnwrappedPose raw{iPose};
  xHat(0) = raw.x;
  xHat(1) = raw.y;
  xHat(2) = raw.h;
  xHat(3) = raw.vf;
  xHat(4) = raw.vs;
  xHat(5) = raw.omega;
  Tracker::setPose(raw);
}

PoseEstimator::State PoseEstimator::f(const PoseEstimator::State &x,
                                      const PoseEstimator::Input &u) {
  const double kT{0.142275};
  const double R{2.3641};
  const double kV{6.30581};
  const int n{4};
  const double rb{0.15113};
  const double rw{0.041275};
  const double J{0.0501};
  const double G{0.75};
  const double m{8.2781};

  const double C1{-(G * G * kT * n) / (kV * R * rw * rw)};
  const double C2{(G * kT * n) / (R * rw)};
  const double D1{2.0 * C1 / m};
  const double D2{C2 / m};
  const double D3{2.0 * rb * rb * C1 / J};
  const double D4{rb * C2 / J};

  State newX{x};
  const double hAdj{M_PI_2 - x(2)};
  newX(0) += dt * (x(3) * std::cos(hAdj) + x(4) * std::sin(hAdj));
  newX(1) += dt * (x(3) * std::sin(hAdj) - x(4) * std::cos(hAdj));
  newX(2) += dt * x(5);
  newX(3) += dt * (D1 * x(3) + D2 * (u(0) + u(1)));
  newX(4) += dt * D1 * x(4);
  newX(5) += dt * (D3 * x(5) + D4 * (u(0) - u(1)));

  return newX;
}

PoseEstimator::Output PoseEstimator::h(const PoseEstimator::State &x,
                                       const PoseEstimator::Input &u) {
  return {x(3), x(5)};
}

PoseEstimator::Input PoseEstimator::getInput() {
  const auto [lV, rV] = drive->getVoltageLR();
  return {lV, rV};
}

PoseEstimator::Output PoseEstimator::getOutput() {
  const double vfDrive{getValueAs<meters_per_second_t>(drive->getVelocity())};
  const double wDrive{
      getValueAs<radians_per_second_t>(drive->getAngularVelocity())};
  return {vfDrive, wDrive};
}

TASK_DEFINITIONS_FOR(PoseEstimator) {
  START_TASK("Pose Estimator Loop", TASK_PRIORITY_MAX)
  while(true) {
    // for(int i{0}; i < 0; i++) {
    predict();
    wait(second_t{dt});
    correct();
    update();
    // }
    // wait(forever);
  }
  END_TASK
}
} // namespace atum