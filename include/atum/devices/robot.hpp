#pragma once

#include "../utility/gui.hpp"
#include <functional>
#include <string>

namespace atum {
using Routine = std::function<void()>;

#define ROUTINE_DEFINITIONS_FOR(robot) void robot::initializeRoutines()
#define START_ROUTINE(name)                                                    \
  if(!routineNames.empty()) routineNames += '\n';                              \
  routineNames += name;                                                        \
        routines.push_back( \
            [this]() {
#define END_ROUTINE                                                            \
  });
#define ROBOT_BOILERPLATE()                                                    \
  public:                                                                      \
  friend class Robot;                                                          \
                                                                               \
  private:                                                                     \
  void initializeRoutines() override

class Robot {
  public:
  Robot() = delete;
  Robot(const Robot &) = delete;
  Robot(Robot &&) = delete;

  template <class SpecRobot>
  Robot(SpecRobot *spec) {
    spec->initializeRoutines();
    GUI::startLoading(spec->getRoutineNames());
  }

  virtual void disabled() = 0;

  virtual void opcontrol() = 0;

  virtual void autonomous();

  std::string getRoutineNames();

  protected:
  virtual void initializeRoutines() = 0;

  std::string routineNames;
  std::vector<Routine> routines;
};

} // namespace atum