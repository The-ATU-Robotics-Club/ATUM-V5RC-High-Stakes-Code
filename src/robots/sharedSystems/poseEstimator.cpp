#include "atum/depend/units.h"
#include "atum/pose/pose.hpp"
#include "poseEstimator.hpp"

namespace atum {
PoseEstimator::PoseEstimator(Drive *iDrive,
                             const PoseEstimator::StateCovariance &iP,
                             const PoseEstimator::StateCovariance &iQ,
                             const PoseEstimator::OutputCovariance &iR) :
    EKF{iP, iQ, iR},
    Tracker{Logger::Level::Debug},
    Task(this, Logger::Level::Info),
    drive{iDrive} {
  const double kT{0.142275};
  const double R{2.61};
  const double kV{6.30581};
  const int n{4};
  const double rb{0.15113};
  const double rw{0.041275};
  const double J{0.1001};
  const double G{0.75};
  const double m{8.2781};

  const double C1{-(G * G * kT * n) / (kV * R * rw * rw)};
  const double C2{(G * kT * n) / (R * rw)};
  D1 = 2.0 * C1 / m;
  D2 = C2 / m;
  D3 = 2.0 * rb * rb * C1 / J;
  D4 = rb * C2 / J;
}

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
  xHatPriori(0) = raw.x;
  xHatPriori(1) = raw.y;
  xHatPriori(2) = raw.h;
  xHatPriori(3) = raw.vf;
  xHatPriori(4) = raw.vs;
  xHatPriori(5) = raw.omega;
  Tracker::setPose(raw);
}

PoseEstimator::State PoseEstimator::f(const PoseEstimator::State &x,
                                      const PoseEstimator::Input &u) {
  State newX{x};
  newX(0) += dt * (x(3) * std::sin(x(2)) + x(4) * std::cos(x(2)));
  newX(1) += dt * (x(3) * std::cos(x(2)) - x(4) * std::sin(x(2)));
  newX(2) += dt * x(5);
  newX(3) += dt * (D1 * x(3) + D2 * (u(0) + u(1)));
  newX(4) += dt * D1 * x(4);
  newX(5) += dt * (D3 * x(5) + D4 * (u(0) - u(1)));

  return newX;
}

PoseEstimator::Output PoseEstimator::h(const PoseEstimator::State &x) {
  return {x(3), x(5)};
}

PoseEstimator::JacobianF PoseEstimator::linearF(const PoseEstimator::State &x,
                                                const PoseEstimator::Input &u) {
  const double h{x(2)};
  const double vf{x(3)};
  const double vs{x(4)};
  return JacobianF{
      {1, 0, (vf * cos(h) - vs * sin(h)) * dt, sin(h) * dt, cos(h) * dt, 0},
      {0, 1, (-vf * sin(h) - vs * cos(h)) * dt, cos(h) * dt, -sin(h) * dt, 0},
      {0, 0, 1, 0, 0, dt},
      {0, 0, 0, 1 + D1 * dt, 0, 0},
      {0, 0, 0, 0, 1 + D1 * dt, 0},
      {0, 0, 0, 0, 0, 1 + D3 * dt}};
}

PoseEstimator::JacobianH PoseEstimator::linearH(const PoseEstimator::State &x) {
  return JacobianH{{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
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