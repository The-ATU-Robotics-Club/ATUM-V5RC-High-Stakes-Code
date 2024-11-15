#pragma once

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

class Robot {
  public:
  Robot() = delete;

  template <typename SpecRobot>
  Robot(SpecRobot *spec) {
    spec->initializeRoutines();
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