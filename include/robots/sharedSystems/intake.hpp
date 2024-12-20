#pragma once

#include "atum/atum.hpp"

namespace atum {
/**
 * @brief The various states that the intake can be in.
 *
 */
enum class IntakeState { Idle, Intaking, Indexing, Outtaking, Jammed, Sorting };

/**
 * @brief Class to implement the intake for the robot. Contains basic controls
 * as well as more complex support for color sorting, anti-jam, and indexing.
 *
 */
class Intake : public Task, public StateMachine<IntakeState> {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

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
    // The time the intake will run outward when throwing while sorting.
    second_t sortThrowTime;
    // The time the intake will attempt to perform an action before giving up.
    second_t generalTimeout;
  };

  /**
   * @brief Constructs a new Intake object with the provided parameters.
   *
   * @param iMtr
   * @param iColorSensor
   * @param iParams
   * @param loggerLevel
   */
  Intake(std::unique_ptr<Motor> iMtr,
         std::unique_ptr<ColorSensor> iColorSensor,
         const Parameters &iParams,
         const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Tell the intake to run inward. Can be interrupted
   * by the anti-jam or color sort code.
   *
   */
  void intake();

  /**
   * @brief Tell the intake to run inward until it detects a ring
   * that isn't being sorted. Can be interrupted by the anti-jam
   * or color sort code.
   *
   */
  void index();

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
  /**
   * @brief This method runs whenever we are intaking. It checks for jams or
   * when to color sort and changes state accordingly.
   *
   */
  void intaking();

  /**
   * @brief This method runs whenever we need to get the intake unjammed. When
   * finished, it goes back to intaking (since that's the only state
   * jammed is accessible from).
   *
   */
  void unjamming();

  /**
   * @brief This method runs whenever we need to sort a ring. When
   * finished, it goes back to intaking (since that's the only state
   * sorting is accessible from).
   *
   */
  void sorting();

  /**
   * @brief For internal use, doesn't have the check on state before switching
   * to intaking/indexing.
   *
   * @param newState
   */
  void forceIntake(const IntakeState newState);

  std::unique_ptr<Motor> mtr;
  std::unique_ptr<ColorSensor> colorSensor;
  Logger logger;
  Parameters params;
  bool antiJamEnabled{true};
  ColorSensor::Color sortOutColor{ColorSensor::Color::Red};
  IntakeState returnState{IntakeState::Intaking};
};
} // namespace atum