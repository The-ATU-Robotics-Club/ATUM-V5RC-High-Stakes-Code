#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Skills")
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