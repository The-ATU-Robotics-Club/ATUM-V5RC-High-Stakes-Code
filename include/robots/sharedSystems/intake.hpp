/**
 * @file intake.hpp
 * @brief Includes the Intake class.
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "atum/atum.hpp"
#include "ladybrown.hpp"

namespace atum {
class Ladybrown;

/**
 * @brief The various states that the intake can be in.
 *
 */
enum class IntakeState {
  Idle,
  Intaking,
  Indexing,
  Outtaking,
  Loading,
  PressLoading,
  Pressed,
  UnpressLoading,
  FinishedLoading
};

/**
 * @brief Class to implement the intake for the robot. Contains basic controls
 * as well as more complex support for color sorting, anti-jam, indexing, and
 * loading.
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
    // The time the intake will continue turning when it sees a ring and is
    // loading.
    second_t pressLoadTime;
    // The amount the intake backs up after loading a ring in the ladybrown.
    degree_t backupFromLoad;
    // The time the intake will attempt to perform an action before giving up.
    second_t generalTimeout;
    double intakingVoltage{12.0};
    double indexingVoltage{12.0};
  };

  /**
   * @brief Constructs a new Intake object with the provided parameters.
   *
   * @param iMotor
   * @param iColorSensor
   * @param iLadybrown
   * @param iParams
   * @param loggerLevel
   */
  Intake(std::unique_ptr<Motor> iMotor,
         std::unique_ptr<ColorSensor> iColorSensor,
         Ladybrown *iLadybrown,
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
   * @brief Tell the intake to work with the ladybrown to load a ring into
   the
   * arm.
   *
   */
  void load();

  /**
   * @brief Tells the intake to backup and finish loading.
   *
   */
  void finishLoading();

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
   * @brief Sets the color the intake will sort out. Setting to "None" will
   * disable sorting.
   *
   * @param iSortOutColor
   */
  void setSortOutColor(const ColorSensor::Color iSortOutColor);

  /**
   * @brief Gets the color the intake is sorting out.
   *
   * @return ColorSensor::Color
   */
  ColorSensor::Color getSortOutColor() const;

  /**
   * @brief Gets the color seen by the color sensor. Returns none if it isn't
   * working.
   *
   * @return ColorSensor::Color
   */
  ColorSensor::Color getColor() const;

  private:
  std::unique_ptr<Motor> motor;
  std::unique_ptr<ColorSensor> colorSensor;
  Ladybrown *ladybrown;
  Logger logger;
  Parameters params;
  ColorSensor::Color sortOutColor{ColorSensor::Color::Red};
  double voltage;
};
} // namespace atum