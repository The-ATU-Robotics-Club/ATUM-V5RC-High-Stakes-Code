#pragma once

#include "atum/atum.hpp"

namespace atum {
/**
 * @brief Class to implement the intake for the robot. Contains basic controls
 * as well as more complex support for color sorting, anti-jam, and positioning
 * the hooks.
 *
 */
class Intake : public Task {
  TASK_BOILERPLATE();

  public:
  /**
   * @brief Different parameters to customize for the intake to
   * function well.
   *
   */
  struct Parameters {
    revolutions_per_minute_t jamVelocity;
    // Timer to track from when we start intaking to when we check for jams.
    Timer timerUntilJamChecks;
    // The time the intake will run outward when jammed.
    second_t timeUntilUnjammed;
  };

  /**
   * @brief Constructs a new Intake object with the provided parameters.
   *
   * @param iMtr
   * @param iParams
   * @param loggerLevel
   */
  Intake(std::unique_ptr<Motor> iMtr,
         const Parameters &iParams,
         const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Tell the intake to run inward. Can be interrupted
   * by the anti-jam or color sort code.
   *
   */
  void intake();

  /**
   * @brief Tell the intake run outward.
   *
   */
  void outtake();

  /**
   * @brief Tell the intake to stop moving.
   *
   */
  void stop();

  /**
   * @brief Sets whether to use the anti-jam.
   *
   * @param iAntiJamEnabled
   */
  void setAntiJam(const bool iAntiJamEnabled);

  private:
  enum class IntakeState { Idle, Intaking, Outtaking, Jammed };

  std::unique_ptr<Motor> mtr;
  Logger logger;
  Parameters params;
  bool antiJamEnabled{true};
  IntakeState state{IntakeState::Idle};
};
} // namespace atum