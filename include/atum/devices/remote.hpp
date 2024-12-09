#pragma once

#include "../devices/motor.hpp"
#include "../time/task.hpp"
#include "../time/time.hpp"
#include "../utility/units.hpp"
#include "api.h"
#include <queue>

namespace atum {
class Remote : public Task {
  public:
  enum class Type {
    Master = pros::controller_id_e_t::E_CONTROLLER_MASTER,
    Partner = pros::controller_id_e_t::E_CONTROLLER_PARTNER
  };

  enum class Button {
    A = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A,
    B = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B,
    X = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X,
    Y = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y,
    Up = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP,
    Down = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN,
    Left = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT,
    Right = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT,
    L1 = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1,
    L2 = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2,
    R1 = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1,
    R2 = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2
  };

  struct StickAxis {
    double x;
    double y;
  };

  Remote(const Type type = Type::Master,
         const Logger::Level loggerLevel = Logger::Level::Info);

  int getLTrigger();

  int getRTrigger();

  StickAxis getLStick();

  StickAxis getRStick();

  bool getPress(const Button button);

  bool getHold(const Button button);

  void print(const std::uint8_t line, const std::string &message);

  void rumble(const std::string &pattern);

  std::int32_t getBattery();

  private:
  TASK_BOILERPLATE();

  pros::Controller remote;
  std::array<std::queue<std::string>, 3> rowQueues;
  std::array<pros::Mutex, 3> rowQueueMutexes;
  static constexpr double deadzone{1.0};
  static constexpr double analogToVolt{Motor::maxVoltage / 127.0};
  static const std::string linePadding;
  static constexpr std::size_t printQueueSize{3};
  static constexpr second_t minimumPrintDelay{75_ms};
};
} // namespace atum