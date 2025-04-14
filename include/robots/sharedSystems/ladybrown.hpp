/**
 * @file ladybrown.hpp
 * @brief Includes the Ladybrown class.
 * @date 2024-12-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "atum/atum.hpp"

namespace atum {
/**
 * @brief The various states that the ladybrown can be in.
 *
 */
enum class LadybrownState {
  Resting,    // The ladybrown is completely down.
  Idle,       // The ladybrown is not moving, but is not necessarily down.
  Settling,   // The ladybrown is coming to a stop, but may still be moving.
  Extending,  // The ladybrown is manually extending.
  Retracting, // The ladybrown is manually retracting.
  MovingTo,   // The ladybrown is moving to a given position.
  Loading     // The ladybrown is ready for a ring to be loaded.
};

/**
 * @brief Class to implement the ladybrown for the robot. Includes manual
 * controls as well as macros to move to several important positions.
 * Additionally provides a holding mechanism.
 *
 */
class Ladybrown : public Task, public StateMachine<LadybrownState> {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  /**
   * @brief Different parameters to customize for the ladybrown to function
   * well.
   *
   */
  struct Parameters {
    double manualVoltage;
    // The lower and upper bounds for movement of the ladybrown.
    std::pair<degree_t, degree_t> bounds;
    degree_t loadingPosition;
    // Used to hold the arm in place (or move to a position if profile follower
    // failed to do so).
    PID holdController{{}};
    // Constant for overcoming gravity (overestimating will make the arm "float"
    // use lowest value to keep arm up).
    double kG;
    // Used to limit jerk.
    SlewRate manualSlew{0};
    // Used to determine when the ladybrown is done moving.
    AcceptableAngle acceptable;
    // Below this RPM, the ladybrown is considered still
    revolutions_per_minute_t stillRPM;
    // Closer than this distance, a ring is considered present when loading.
    meter_t loadRingDistance;
    // Closer than this distance, a ring is considered present when indexing.
    meter_t indexRingDistance;
  };

  /**
   * @brief Constructs a new Ladybrown object with the provided parameters.
   *
   * @param iMotor
   * @param iDistance
   * @param iRotation
   * @param iParams
   * @param loggerLevel
   */
  Ladybrown(std::unique_ptr<Motor> iMotor,
            std::unique_ptr<DistanceSensor> iDistance,
            std::unique_ptr<RotationSensor> iRotation,
            const Parameters &iParams,
            const Logger::Level loggerLevel = Logger::Level::Info);

  /**
   * @brief Tells the ladybrown to go back to its lowered position (where it
   * started).
   *
   */
  void rest();

  /**
   * @brief Tells the ladybrown to stop moving and (in manual control) sets the
   * position to hold.
   *
   */
  void stop();

  /**
   * @brief Tells the ladybrown to manually extend.
   *
   */
  void extend();

  /**
   * @brief Tells the ladybrown to manually retract.
   *
   */
  void retract();

  /**
   * @brief Tells the ladybrown to move to its loading position.
   *
   */
  void load();

  /**
   * @brief Tells the ladybrown to move to a certain position.
   *
   * @param iTarget
   */
  void moveTo(const degree_t iTarget);

  /**
   * @brief Returns if the ladybrown sees a ring in the ladybrown carriage based
   * on the distance sensor and if it is resting.
   *
   * @return true
   * @return false
   */
  bool ringInCarriage() const;

  /**
   * @brief Returns if the ladybrown sees a ring in the indexer based on the
   * distance sensor and if it is resting.
   *
   * @return true
   * @return false
   */
  bool ringInIndexer() const;

  /**
   * @brief Returns if the distance sensor is functional or not.
   *
   * @return true
   * @return false
   */
  bool checkRingDetection() const;

  /**
   * @brief Gets the position of the arm based on the rotation sensor if
   * available and the motor.
   *
   * @return degree_t
   */
  degree_t getPosition() const;


  private:
  /**
   * @brief Moves to the position associated with a given state, so long as the
   * current state does not change.
   *
   */
  void moveToControls();

  std::unique_ptr<Motor> motor;
  std::unique_ptr<DistanceSensor> distance;
  std::unique_ptr<RotationSensor> rotation;
  Parameters params;
  Logger logger;
  degree_t target;
  double voltage;
  std::optional<LadybrownState> nextState;
};
} // namespace atum