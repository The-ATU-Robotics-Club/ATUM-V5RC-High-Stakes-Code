/**
 * @file poseEstimator.hpp
 * @brief Includes the PoseEstimator class.
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "atum/atum.hpp"

namespace atum {
/**
 * @brief This class incorporates a model of the robot's differential drive with
 * given sensors using an EKF.
 *
 */
class PoseEstimator : public EKF<6, 2, 8>, public Tracker, public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  /**
   * @brief Constructs a new PoseEstimator.
   *
   * @param iDrive
   * @param iOdometry
   * @param iOTOS
   * @param iP
   * @param iQ
   * @param iR
   * @param loggerLevel
   */
  PoseEstimator(Drive *iDrive,
                std::unique_ptr<Odometry> iOdometry,
                std::unique_ptr<OTOS> iOTOS,
                const PoseEstimator::StateCovariance &iP,
                const PoseEstimator::StateCovariance &iQ,
                const PoseEstimator::OutputCovariance &iR,
                const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Updates the current pose to the estimate by the EKF.
   *
   * @return Pose
   */
  Pose update() override;

  /**
   * @brief Sets the internal pose as well as the pose of the odometers and OTOS. 
   * 
   * @param iPose 
   */
  void setPose(const Pose &iPose) override;

  private:
  static constexpr double dt{0.01};

  /**
   * @brief Encapsulates the model of the drive. 
   * 
   * @param x 
   * @param u 
   * @return State 
   */
  State f(const State &x, const Input &u) override;

  /**
   * @brief Determines how to transfer state estiamtes to sensor readings. 
   * 
   * @param x 
   * @return Output 
   */
  Output h(const State &x) override;

  /**
   * @brief A Jacobian representation of f, evaluated at x and u.
   * 
   * @param x 
   * @param u 
   * @return JacobianF 
   */
  JacobianF linearF(const State &x, const Input &u) override;

  /**
   * @brief A Jacobian representation of h, evaluated at x and u.
   * 
   * @param x 
   * @return JacobianH 
   */
  JacobianH linearH(const State &x) override;

  /**
   * @brief Gets the voltage applied to both sides of the drive.
   * 
   * @return Input 
   */
  Input getInput() override;

  /**
   * @brief Gets the output of the odometers, OTOS, and drive motor encoders. 
   * 
   * @return Output 
   */
  Output getOutput() override;

  /**
   * @brief Manages setting pose internally (i.e., updating the EKF appropriately). 
   * 
   * @param iPose 
   */
  void setPoseInternal(const Pose &iPose);

  Drive *drive;
  std::unique_ptr<Odometry> odometry;
  std::unique_ptr<OTOS> otos;

  double D1, D2, D3, D4;
};
} // namespace atum