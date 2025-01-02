#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Skills")
  Trajectory::setDefaultParams({7, 76.5_in_per_s, 76.5_in_per_s_sq, 11.825_in});
  Trajectory::Parameters testingParams;
  testingParams.spacing = 1_in;
  Trajectory test{{{-1_m, 0_m, 0_deg}, {1_m, 0_m, 0_deg}},
                  testingParams,
                  Logger::Level::Debug};
  END_ROUTINE
  START_ROUTINE("Test 1")
  intake->load();
  wait(5_s);
  intake->stop();
  END_ROUTINE
  START_ROUTINE("Test 2")
  END_ROUTINE
  START_ROUTINE("Test 3")
  END_ROUTINE
}
} // namespace atum