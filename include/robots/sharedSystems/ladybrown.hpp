#pragma once

#include "atum/atum.hpp"

namespace atum {
enum class LadybrownState {
  Idle,
  Extending,
  Retracting,
  Resting,
  Loading,
  Preparing,
  Scoring,
  FinishScoring
};

class Ladybrown : public Task, public StateMachine<LadybrownState> {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  struct Parameters {
    double maxVoltage;
    // The angle of the arms to the floor (completely up would be 90 deg; think
    // unit circle).
    degree_t absoluteStartingPosition{0_deg};
    degree_t noHoldPosition{0_deg};
    degree_t flippingPosition{0_deg};
    // Map connecting states that involve moving to a position to their
    // corresponding end position.
    std::unordered_map<LadybrownState, degree_t> statePositions;
    // Amount of time allocated for the pistons to move before the ladybrown
    // moves on.
    second_t pistonDelay{0_s};
    // Constant for overcoming gravity (overestimating will make the arm "float"
    // use lowest value to keep arm up).
    double kG{0.0};
    PID holdController{{}};
    PID balanceController{{}};
    // The time the intake will attempt to perform an action before giving up.
    second_t generalTimeout{forever};
  };

  Ladybrown(std::unique_ptr<Motor> iLeft,
            std::unique_ptr<Motor> iRight,
            std::unique_ptr<Piston> iPiston,
            std::unique_ptr<RotationSensor> iRotation,
            std::unique_ptr<LineTracker> iLine,
            const Parameters &iParams,
            std::unique_ptr<AngularProfileFollower> iFollower,
            const Logger::Level loggerLevel = Logger::Level::Info);

  void stop();

  void extend();

  void retract();

  void rest();

  void prepare();

  void load();

  void score();

  bool mayConflictWithIntake();

  LadybrownState getClosestPosition() const;

  bool hasRing() const;

  private:
  void moveTo(const degree_t target);
  void setVoltage(const double voltage);
  double getVoltage();

  std::unique_ptr<Motor> left;
  std::unique_ptr<Motor> right;
  std::unique_ptr<Piston> piston;
  std::unique_ptr<RotationSensor> rotation;
  std::unique_ptr<LineTracker> line;
  Parameters params;
  std::unique_ptr<AngularProfileFollower> follower;
  Logger logger;
  std::optional<degree_t> holdPosition;

  double voltage;
  pros::Mutex voltageMutex;
};
} // namespace atum