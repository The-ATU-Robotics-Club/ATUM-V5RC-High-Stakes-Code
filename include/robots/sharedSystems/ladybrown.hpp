#pragma once

#include "atum/atum.hpp"

namespace atum {
enum class LadybrownStates { Idle, Extending, Retracting, Loading, Scoring };

class Ladybrown : public Task, StateMachine<LadybrownStates> {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  struct Parameters {
    double maxVoltage;
    double restPosition;
    double loadingPosition;
    double flippingPosition;
    double scoredPosition;
    AcceptableAngle acceptableDeviation;
    SlewRate voltageSlew;
    PID positionController;
    PID balanceController;
  };

  Ladybrown(std::unique_ptr<Motor> iLeft,
            std::unique_ptr<Motor> iRight,
            std::unique_ptr<Piston> iPiston,
            std::unique_ptr<RotationSensor> iRotation,
            std::unique_ptr<LineTracker> iLine,
            const Parameters &iParams,
            const Logger::Level loggerLevel = Logger::Level::Info);

  void stop();

  void extend();

  void retract();

  void load();

  void score();

  private:
  void setVoltage(const double voltage);
  double getVoltage();

  std::unique_ptr<Motor> left;
  std::unique_ptr<Motor> right;
  std::unique_ptr<Piston> piston;
  std::unique_ptr<RotationSensor> rotation;
  std::unique_ptr<LineTracker> line;
  Parameters params;
  Logger logger;

  double voltage;
  pros::Mutex voltageMutex;
};
} // namespace atum