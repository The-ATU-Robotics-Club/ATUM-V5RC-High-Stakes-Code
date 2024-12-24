#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Test 1")
  // Remember that these can go off screen! Need to change series ranges!
  atum::LateralProfile::Parameters fourStages{10_mps, 10_mps_sq, 0.1_mps_cb};
  atum::LateralProfile::Parameters fiveStages{0.1_mps, 0.1_mps_sq, 0.1_mps_cb};
  atum::LateralProfile::Parameters sixStages{10_mps, 0.1_mps_sq, 0.1_mps_cb};
  atum::LateralProfile::Parameters allStages{0.1_mps, 0.1_mps_sq, 0.2_mps_cb};
  atum::LateralProfile::Parameters trapezoidal{0.1_mps, 0.1_mps_sq};
  atum::LateralProfile mp{0_m, 0.5_m, allStages, atum::Logger::Level::Debug};
  const Pose start{drive->getPose()};
  while(true) {
    atum::LateralProfile::Point p = mp.getPoint(distance(start, drive->getPose()));
    atum::wait();
  }
  END_ROUTINE
  START_ROUTINE("Test 2")
  END_ROUTINE
  START_ROUTINE("Test 3")
  END_ROUTINE
}
} // namespace atum