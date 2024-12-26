#pragma once

#include "atum/atum.hpp"

namespace atum {
enum class LadybrownStates { Idle, Extending, Retracting, Loading, Scoring };

class Ladybrown : public Task, StateMachine<LadybrownStates> {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  struct Parameters {
    double maxVoltage;
    degree_t restPosition;
    degree_t loadingPosition;
    degree_t flippingPosition;
    degree_t scoredPosition;
    PID holdController;
    PID balanceController;
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

  void load();

  void score();

  private:
  void setVoltage(const double voltage);
  double getVoltage();
  void moveTo(const degree_t target);

  std::unique_ptr<Motor> left;
  std::unique_ptr<Motor> right;
  std::unique_ptr<Piston> piston;
  std::unique_ptr<RotationSensor> rotation;
  std::unique_ptr<LineTracker> line;
  Parameters params;
  std::unique_ptr<AngularProfileFollower> follower;
  Logger logger;

  double voltage;
  pros::Mutex voltageMutex;
};
} // namespace atum